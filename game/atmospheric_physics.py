"""Prototype SI-unit fighter forces; zero wind, constant density, no aero moments."""

from dataclasses import dataclass, field
import math

import numpy as np

from game.flight_state import VehicleMode
from game.vtol_physics import VTOL, engine_force, maneuver_force


@dataclass(frozen=True)
class AircraftParameters:
    mass: float = 6000.0  # kg
    wing_area: float = 30.0  # m²
    air_density: float = 1.225  # kg/m³
    gravity: float = 9.81  # m/s²
    max_engine_thrust: float = 120000.0  # N
    zero_lift_drag_coefficient: float = 0.05
    induced_drag_factor: float = 0.06
    lift_curve_slope: float = 4.5  # CL / radian
    max_lift_coefficient: float = 1.4
    high_aoa_warning_fraction: float = 0.8
    stall_angle_deg: float = 24.0
    stall_decay_width_deg: float = 30.0
    stall_drag_coefficient: float = 0.45
    pitch_authority_deg: float = 60.0  # deg/s at reference speed
    roll_authority_deg: float = 90.0
    yaw_authority_deg: float = 40.0
    reference_airspeed: float = 100.0  # m/s
    minimum_stall_control_factor: float = 0.35
    test_altitude: float = 1000.0  # m
    test_airspeed: float = 120.0  # m/s
    ground_altitude: float = 0.0  # m
    crash_speed: float = 5.0  # m/s; slower contact is frozen as GROUNDED


PARAMETERS = AircraftParameters()


def zero_vector():
    return np.zeros(3, dtype=np.float64)


@dataclass
class AeroForces:
    airspeed: float = 0.0
    dynamic_pressure: float = 0.0
    alpha: float = 0.0  # radians; positive when nose above flight path
    beta: float = 0.0  # radians; positive velocity toward local right
    lift_coefficient: float = 0.0
    drag_coefficient: float = 0.0
    stall_effectiveness: float = 1.0
    lift_force: np.ndarray = field(default_factory=zero_vector)
    drag_force: np.ndarray = field(default_factory=zero_vector)
    thrust_force: np.ndarray = field(default_factory=zero_vector)
    gravity_force: np.ndarray = field(default_factory=zero_vector)
    maneuver_force: np.ndarray = field(default_factory=zero_vector)

    @property
    def total_force(self):
        return self.lift_force + self.drag_force + self.thrust_force + self.gravity_force + self.maneuver_force


def coefficients(alpha, parameters=PARAMETERS):
    stall_excess = max(0.0, abs(alpha) - math.radians(parameters.stall_angle_deg))
    width = math.radians(parameters.stall_decay_width_deg)
    # A wider decay retains useful lift in combat AoA without unbounded CL.
    stall_effectiveness = math.exp(-(stall_excess / width) ** 2)
    linear_lift = parameters.lift_curve_slope * alpha
    limited_lift = np.clip(linear_lift, -parameters.max_lift_coefficient, parameters.max_lift_coefficient)
    lift_coefficient = float(limited_lift) * stall_effectiveness
    drag_coefficient = (parameters.zero_lift_drag_coefficient
                        + parameters.induced_drag_factor * lift_coefficient ** 2
                        + parameters.stall_drag_coefficient * (1 - stall_effectiveness))
    return lift_coefficient, drag_coefficient, stall_effectiveness


def calculate_forces(vehicle, throttle=None, velocity=None, parameters=PARAMETERS,
                     controls=None, vector=None):
    """Pure force evaluation. Incoming wind is opposite relative aircraft velocity."""
    if vehicle.flight_state.mode is VehicleMode.BATTLEDROID:
        from game.battledroid_physics import forces
        from types import SimpleNamespace
        return forces(vehicle, controls if controls is not None else SimpleNamespace(strafe=0.,lift=0.),
                      throttle=throttle,velocity=velocity)
    air_velocity = vehicle.velocity if velocity is None else np.asarray(velocity, dtype=float)
    if not np.isfinite(air_velocity).all() or not np.isfinite(vehicle.orientation).all():
        raise ValueError("Atmospheric state must be finite.")
    throttle = vehicle.throttle if throttle is None else throttle
    if not math.isfinite(throttle):
        raise ValueError("Throttle must be finite.")
    result = AeroForces()
    vtol = vehicle.flight_state.mode is VehicleMode.VTOL
    if vtol:
        result.thrust_force = engine_force(vehicle, throttle, vector)
        result.maneuver_force = maneuver_force(vehicle, controls)
    else:
        result.thrust_force = vehicle.forward * parameters.max_engine_thrust * np.clip(throttle, 0, 1)
    result.gravity_force = np.array((0.0, -parameters.mass * parameters.gravity, 0.0))
    # Wind is currently zero. Later subtract world-space wind here.
    result.airspeed = float(np.linalg.norm(air_velocity))
    if not math.isfinite(result.airspeed):
        raise ValueError("Airspeed must be finite.")
    result.dynamic_pressure = 0.5 * parameters.air_density * result.airspeed ** 2
    if result.airspeed < 1e-6:
        return result  # no undefined airflow normalization or aerodynamic forces
    local_forward = float(np.dot(air_velocity, vehicle.forward))
    local_up = float(np.dot(air_velocity, vehicle.up))
    local_right = float(np.dot(air_velocity, vehicle.right))
    result.alpha = math.atan2(-local_up, local_forward)
    result.beta = math.atan2(local_right, math.hypot(local_forward, local_up))
    result.lift_coefficient, result.drag_coefficient, result.stall_effectiveness = coefficients(result.alpha, parameters)
    if vtol:
        # Deployed structure retains a little wing lift but presents much more drag.
        result.lift_coefficient *= VTOL.wing_lift_factor
        result.drag_coefficient *= VTOL.drag_factor
    airflow_direction = air_velocity / result.airspeed
    # Span cross flight-path gives lift perpendicular to flow, banked with aircraft.
    lift_direction = np.cross(vehicle.right, airflow_direction)
    lift_length = float(np.linalg.norm(lift_direction))
    if lift_length > 1e-8:
        lift_direction /= lift_length
        result.lift_force = lift_direction * result.dynamic_pressure * parameters.wing_area * result.lift_coefficient
    result.drag_force = -airflow_direction * result.dynamic_pressure * parameters.wing_area * result.drag_coefficient
    if not np.isfinite(result.total_force).all():
        raise ValueError("Atmospheric forces became nonfinite.")
    return result


def control_authority(forces, parameters=PARAMETERS):
    reference_pressure = 0.5 * parameters.air_density * parameters.reference_airspeed ** 2
    pressure_factor = 1.25 * forces.dynamic_pressure / (forces.dynamic_pressure + 0.25 * reference_pressure)
    stall_factor = (parameters.minimum_stall_control_factor
                    + (1 - parameters.minimum_stall_control_factor) * forces.stall_effectiveness)
    return pressure_factor * stall_factor


def reset_test_flight(vehicle, parameters=PARAMETERS):
    """Trim a level flight path with slight nose-up incidence and balanced thrust."""
    from game.flight_state import Environment, FlightStatus, VehicleMode

    q_area = 0.5 * parameters.air_density * parameters.test_airspeed ** 2 * parameters.wing_area
    low, high = 0.0, math.radians(parameters.stall_angle_deg)
    for _ in range(50):
        alpha = (low + high) / 2
        cl, cd, _ = coefficients(alpha, parameters)
        # Engine thrust balances drag horizontally; its vertical part assists lift.
        vertical_force = q_area * (cl + cd * math.tan(alpha))
        if vertical_force < parameters.mass * parameters.gravity:
            low = alpha
        else:
            high = alpha
    alpha = (low + high) / 2
    _, cd, _ = coefficients(alpha, parameters)
    vehicle.position[:] = (0, parameters.test_altitude, 0)
    vehicle.forward[:] = (0, 0, -1)
    vehicle.right[:] = (1, 0, 0)
    vehicle.up[:] = (0, 1, 0)
    vehicle.rotate(pitch=alpha)
    vehicle.velocity[:] = (0, 0, -parameters.test_airspeed)
    vehicle.throttle = float(np.clip(q_area * cd / math.cos(alpha) / parameters.max_engine_thrust, 0, 1))
    vehicle.engine_throttle = vehicle.throttle
    vehicle.thrust_vector = 0.0
    vehicle.acceleration[:] = 0
    vehicle.pitch_rate = vehicle.yaw_rate = vehicle.roll_rate = 0.0
    vehicle.flight_state.environment = Environment.ATMOSPHERE
    vehicle.flight_state.mode = VehicleMode.FIGHTER
    vehicle.flight_state.status = FlightStatus.FLYING
    vehicle.aerodynamics = calculate_forces(vehicle, parameters=parameters)
