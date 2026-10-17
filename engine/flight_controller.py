"""Pilot commands with separate inertial space and aerodynamic fighter integration."""

import math

import numpy as np

from game.atmospheric_physics import (
    PARAMETERS,
    AeroForces,
    calculate_forces,
    control_authority,
    reset_test_flight,
)
from game.flight_control_computer import FlightControlComputer
from game.flight_state import Environment, FlightStatus, VehicleMode
from game.vtol_physics import VTOL, engine_force, maneuver_force

MAX_THRUST = 12.0  # m/s² (SPACE acceleration, not engine force)
TRANSLATION_ACCELERATION = 8.0
PITCH_RATE = math.radians(60.0)  # radians / second
YAW_RATE = math.radians(60.0)
ROLL_RATE = math.radians(90.0)
DAMPING = 0.35  # temporary artificial space damping / second
BRAKE_DAMPING = 2.0  # additional damping while X is held
THROTTLE_RATE = 0.5  # throttle fraction / second (zero to full in two seconds)
MAX_STEP = 1.0 / 120.0  # small steps keep turning thrust consistent across frame rates


class FlightController:
    """Integrate pilot commands while coordinating configuration-specific ground contact."""

    def __init__(self, vehicle, ground_environment=None):
        """Bind a vehicle, flight computer, and optional shared ground queries.

        Args:
            vehicle: Vehicle whose pose, motion, and configuration provide simulation state.
            ground_environment: Shared height/normal and obstacle queries, or None for default
                ground.
        """
        self.vehicle = vehicle
        self.fcc = FlightControlComputer()
        from engine.battledroid_controller import BattledroidController

        self.battledroid = BattledroidController(vehicle, self.fcc)
        from engine.vtol_ground import VTOLGroundContact

        self.vtol_ground = VTOLGroundContact(vehicle, self.fcc)
        from game.fighter_ground import FighterGroundContact

        self.fighter_ground = FighterGroundContact(vehicle)
        self.ground_environment = ground_environment
        self.fighter_ground.environment = ground_environment
        if ground_environment is not None:

            def height(position):
                """Query ground height and copy its support normal into vehicle contact state.

                Args:
                    position: XYZ position in meters in the coordinate frame used by this
                        operation.

                Returns:
                    Ground elevation in meters.
                """
                vehicle.ground_contact.normal[:] = ground_environment.normal(position)
                return ground_environment.height(position)

            self.vtol_ground.ground_height = self.battledroid.ground_height = height

    def reset_atmosphere(self):
        """Reset atmospheric test flight, jump state, airfield state, and hover assistance."""
        self.battledroid.jump_remaining = 0.0
        self.battledroid._jump_down = False
        self.vehicle.airfield.reset()
        reset_test_flight(self.vehicle)
        self.fcc.clear_hover()

    def switch_space(self):
        """Select inertial flight and clear atmospheric force/contact and hover telemetry."""
        self.battledroid.jump_remaining = 0.0
        self.fcc.clear_hover()
        self.vehicle.flight_state.environment = Environment.SPACE
        self.vehicle.flight_state.status = FlightStatus.FLYING
        self.vehicle.aerodynamics = AeroForces()
        self.vehicle.ground_contact.reaction_force[:] = 0
        self.vehicle.ground_contact.friction_force[:] = 0
        self.vehicle.ground_contact.landing_approach = False

    def update(self, dt, controls):
        """Advance vehicle motion and contact using bounded physics substeps.

        Args:
            dt: Elapsed simulation time in seconds.
            controls: Pilot command fields; signed axes are normalized to [-1, 1].
        """
        state = self.vehicle.flight_state
        if state.mode is VehicleMode.BATTLEDROID:
            previous = self.vehicle.position.copy()
            self.battledroid.update(dt, controls)
            if self.ground_environment is not None and dt > 0:
                self.ground_environment.collide(self.vehicle, previous, dt)
                if (
                    state.status is FlightStatus.GROUNDED
                    and not self.ground_environment.landing_suitable(self.vehicle)
                ):
                    self.fighter_ground.crash("UNSAFE SLOPE")
            return
        self.battledroid.jump_remaining = 0.0
        self.battledroid._jump_down = False
        if state.mode not in (VehicleMode.FIGHTER, VehicleMode.VTOL):
            raise NotImplementedError("Unknown vehicle configuration.")
        if not math.isfinite(dt):
            raise ValueError("Delta time must be finite.")
        if state.environment is Environment.ATMOSPHERE:
            self._update_atmosphere(dt, controls)
            return
        if state.environment is not Environment.SPACE:
            raise NotImplementedError("Unknown environment.")
        vtol = state.mode is VehicleMode.VTOL
        rate_limits = dict(
            pitch=math.radians(VTOL.pitch_rate_deg) if vtol else PITCH_RATE,
            yaw=math.radians(VTOL.yaw_rate_deg) if vtol else YAW_RATE,
            roll=math.radians(VTOL.roll_rate_deg) if vtol else ROLL_RATE,
        )
        if dt <= 0:
            return
        steps = max(1, math.ceil(dt / MAX_STEP))
        step = dt / steps
        vtol = state.mode is VehicleMode.VTOL
        damping = DAMPING + (BRAKE_DAMPING if controls.brake and not vtol else 0.0)
        for _ in range(steps):
            command = self.fcc.commands(controls, self.vehicle, step, rate_limits)
            self._apply_rates(command, rate_limits)
            old_throttle = self.vehicle.throttle
            self.vehicle.throttle = max(
                0.0, min(1.0, old_throttle + command.thrust * THROTTLE_RATE * step)
            )
            # Midpoint throttle approximates the gradual ramp, including saturation.
            throttle = (old_throttle + self.vehicle.throttle) / 2
            vector = self._update_vector(step, command)
            # Sample thrust at the middle of the orientation step.
            rotation = dict(
                pitch=self.vehicle.pitch_rate * step / 2,
                yaw=self.vehicle.yaw_rate * step / 2,
                roll=self.vehicle.roll_rate * step / 2,
            )
            self.vehicle.rotate(**rotation)
            if vtol:
                thrust = engine_force(self.vehicle, throttle, vector)
                maneuvers = maneuver_force(self.vehicle, command)
                self.vehicle.acceleration = (thrust + maneuvers) / PARAMETERS.mass
            else:
                self.vehicle.acceleration = (
                    self.vehicle.forward * throttle * MAX_THRUST
                    + self.vehicle.right * command.strafe * TRANSLATION_ACCELERATION
                    + self.vehicle.up * command.lift * TRANSLATION_ACCELERATION
                )
            # Exact integration of dv/dt = acceleration - damping * velocity
            # for constant acceleration within this small step.
            if damping > 0:
                decay = math.exp(-damping * step)
                integral = -math.expm1(-damping * step) / damping
                self.vehicle.position += (
                    self.vehicle.velocity * integral
                    + self.vehicle.acceleration * (step - integral) / damping
                )
                self.vehicle.velocity = (
                    self.vehicle.velocity * decay + self.vehicle.acceleration * integral
                )
            else:
                self.vehicle.position += (
                    self.vehicle.velocity * step + self.vehicle.acceleration * step * step / 2
                )
                self.vehicle.velocity += self.vehicle.acceleration * step
            self.vehicle.rotate(**rotation)

        self.vehicle.engine_throttle = self.vehicle.throttle
        if vtol:
            self.vehicle.aerodynamics = AeroForces(
                airspeed=self.vehicle.speed,
                thrust_force=engine_force(self.vehicle, self.vehicle.throttle),
                maneuver_force=maneuver_force(self.vehicle, command),
            )

    def _apply_rates(self, command, rate_limits):
        """Write angular rates from normalized commands and radian-per-second limits.

        Args:
            command: Bounded pitch/yaw/roll command fields.
            rate_limits: Axis rate limits in radians per second.
        """
        for axis in ("pitch", "yaw", "roll"):
            setattr(self.vehicle, axis + "_rate", getattr(command, axis) * rate_limits[axis])

    def _update_vector(self, step, controls):
        """Advance persistent VTOL thrust vectoring and sample its midpoint.

        Args:
            step: Physics substep duration in seconds.
            controls: Pilot command fields; signed axes are normalized to [-1, 1].

        Returns:
            Midpoint vector fraction; the current fraction outside VTOL.
        """
        if self.vehicle.flight_state.mode is not VehicleMode.VTOL:
            return self.vehicle.thrust_vector
        old_vector = self.vehicle.thrust_vector
        self.vehicle.thrust_vector = max(
            0.0, min(1.0, old_vector + controls.vector_command * VTOL.vector_rate * step)
        )
        return (old_vector + self.vehicle.thrust_vector) / 2

    def _update_atmosphere(self, dt, controls):
        """Integrate atmospheric forces with midpoint orientation and velocity sampling.

        Args:
            dt: Elapsed simulation time in seconds.
            controls: Pilot command fields; signed axes are normalized to [-1, 1].
        """
        vehicle = self.vehicle
        vehicle.airfield.brakes = bool(getattr(controls, "wheel_brake", False))
        vehicle.airfield.brakes_active = False
        if vehicle.flight_state.status is FlightStatus.CRASHED or dt <= 0:
            return
        steps = max(1, math.ceil(dt / MAX_STEP))
        step = dt / steps
        for _ in range(steps):
            if vehicle.flight_state.mode is VehicleMode.FIGHTER:
                vehicle.airfield.update(step)
            current_forces = calculate_forces(vehicle)
            vtol = vehicle.flight_state.mode is VehicleMode.VTOL
            if vtol:
                self.vtol_ground.begin(step)
            else:
                self.fighter_ground.begin()
            authority = 1.0 if vtol else control_authority(current_forces)
            pitch_rate = VTOL.pitch_rate_deg if vtol else PARAMETERS.pitch_authority_deg
            roll_rate = VTOL.roll_rate_deg if vtol else PARAMETERS.roll_authority_deg
            yaw_rate = VTOL.yaw_rate_deg if vtol else PARAMETERS.yaw_authority_deg
            rate_limits = dict(
                pitch=math.radians(pitch_rate) * authority,
                roll=math.radians(roll_rate) * authority,
                yaw=math.radians(yaw_rate) * authority,
            )
            command = self.fcc.commands(controls, vehicle, step, rate_limits)
            previous_yaw_rate = vehicle.yaw_rate
            self._apply_rates(command, rate_limits)
            if not vtol and self.fighter_ground.supported:
                vehicle.yaw_rate = previous_yaw_rate
                vehicle.yaw_rate = self.fighter_ground.steer_rate(step, controls.yaw)
            rotation = dict(
                pitch=vehicle.pitch_rate * step / 2,
                yaw=vehicle.yaw_rate * step / 2,
                roll=vehicle.roll_rate * step / 2,
            )
            old_throttle = vehicle.throttle
            vehicle.throttle = max(
                0.0, min(1.0, old_throttle + command.thrust * THROTTLE_RATE * step)
            )
            throttle = (old_throttle + vehicle.throttle) / 2
            vector = self._update_vector(step, command)
            if command.throttle_override is not None:
                throttle = command.throttle_override
            vehicle.engine_throttle = throttle
            vehicle.rotate(**rotation)
            # Midpoint force integration: drag depends on velocity, not a decay percentage.
            initial_forces = calculate_forces(
                vehicle, throttle=throttle, controls=command, vector=vector
            )
            initial_acceleration = (
                self.vtol_ground.acceleration(initial_forces, vehicle.velocity, step)
                if vtol
                else self.fighter_ground.acceleration(initial_forces, vehicle.velocity, step)
            )
            midpoint_velocity = vehicle.velocity + initial_acceleration * step / 2
            midpoint_forces = calculate_forces(
                vehicle,
                throttle=throttle,
                velocity=midpoint_velocity,
                controls=command,
                vector=vector,
            )
            vehicle.acceleration = (
                self.vtol_ground.acceleration(midpoint_forces, midpoint_velocity, step)
                if vtol
                else self.fighter_ground.acceleration(midpoint_forces, midpoint_velocity, step)
            )
            previous_position = vehicle.position.copy()
            vehicle.position += midpoint_velocity * step
            vehicle.velocity += vehicle.acceleration * step
            vehicle.rotate(**rotation)
            if self.ground_environment is not None:
                self.ground_environment.collide(vehicle, previous_position, step)
                if vehicle.flight_state.status is FlightStatus.CRASHED:
                    break
            if not np.isfinite(vehicle.position).all() or not np.isfinite(vehicle.velocity).all():
                raise ValueError("Atmospheric integration became nonfinite.")
            if vtol:
                self.vtol_ground.finish()
                if (
                    self.ground_environment is not None
                    and vehicle.flight_state.status is FlightStatus.GROUNDED
                    and not self.ground_environment.landing_suitable(vehicle)
                ):
                    self.fighter_ground.crash("UNSAFE SLOPE")
                if vehicle.flight_state.status is FlightStatus.CRASHED:
                    break
            else:
                self.fighter_ground.finish()
                if vehicle.flight_state.status is FlightStatus.CRASHED:
                    self.fcc.clear_hover()
                    break
        if vehicle.flight_state.status is FlightStatus.FLYING or (
            vehicle.flight_state.status is FlightStatus.GROUNDED
            and vehicle.flight_state.mode in (VehicleMode.VTOL, VehicleMode.FIGHTER)
        ):
            vehicle.aerodynamics = calculate_forces(
                vehicle, throttle=vehicle.engine_throttle, controls=command
            )
