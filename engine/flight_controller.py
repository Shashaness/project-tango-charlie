"""Pilot commands with separate inertial space and aerodynamic fighter integration."""

import math

import numpy as np

from game.flight_control_computer import FlightControlComputer
from game.vtol_physics import VTOL, engine_force, maneuver_force
from game.flight_state import Environment, VehicleMode, FlightStatus
from game.atmospheric_physics import (PARAMETERS, AeroForces, calculate_forces,
                                      control_authority, reset_test_flight)

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
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.fcc = FlightControlComputer()
        from engine.battledroid_controller import BattledroidController
        self.battledroid = BattledroidController(vehicle,self.fcc)
        from engine.vtol_ground import VTOLGroundContact
        self.vtol_ground = VTOLGroundContact(vehicle,self.fcc)
        from game.fighter_ground import FighterGroundContact
        self.fighter_ground = FighterGroundContact(vehicle)

    def reset_atmosphere(self):
        self.battledroid.jump_remaining=0.
        self.battledroid._jump_down=False
        reset_test_flight(self.vehicle)
        self.fcc.clear_hover()

    def switch_space(self):
        self.battledroid.jump_remaining=0.
        self.fcc.clear_hover()
        self.vehicle.flight_state.environment = Environment.SPACE
        self.vehicle.flight_state.status = FlightStatus.FLYING
        self.vehicle.aerodynamics = AeroForces()
        self.vehicle.ground_contact.reaction_force[:]=0
        self.vehicle.ground_contact.friction_force[:]=0
        self.vehicle.ground_contact.landing_approach=False

    def update(self, dt, controls):
        state = self.vehicle.flight_state
        if state.mode is VehicleMode.BATTLEDROID:
            self.battledroid.update(dt,controls)
            return
        self.battledroid.jump_remaining=0.
        self.battledroid._jump_down=False
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
            roll=math.radians(VTOL.roll_rate_deg) if vtol else ROLL_RATE)
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
            self.vehicle.throttle = max(0.0, min(1.0, old_throttle + command.thrust * THROTTLE_RATE * step))
            # Midpoint throttle approximates the gradual ramp, including saturation.
            throttle = (old_throttle + self.vehicle.throttle) / 2
            vector = self._update_vector(step, command)
            # Sample thrust at the middle of the orientation step.
            rotation = dict(pitch=self.vehicle.pitch_rate * step / 2,
                            yaw=self.vehicle.yaw_rate * step / 2,
                            roll=self.vehicle.roll_rate * step / 2)
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
                self.vehicle.position += (self.vehicle.velocity * integral
                                         + self.vehicle.acceleration * (step - integral) / damping)
                self.vehicle.velocity = self.vehicle.velocity * decay + self.vehicle.acceleration * integral
            else:
                self.vehicle.position += self.vehicle.velocity * step + self.vehicle.acceleration * step * step / 2
                self.vehicle.velocity += self.vehicle.acceleration * step
            self.vehicle.rotate(**rotation)

        self.vehicle.engine_throttle = self.vehicle.throttle
        if vtol:
            self.vehicle.aerodynamics = AeroForces(
                airspeed=self.vehicle.speed,
                thrust_force=engine_force(self.vehicle, self.vehicle.throttle),
                maneuver_force=maneuver_force(self.vehicle, command))

    def _apply_rates(self, command, rate_limits):
        for axis in ('pitch', 'yaw', 'roll'):
            setattr(self.vehicle, axis + '_rate', getattr(command, axis) * rate_limits[axis])

    def _update_vector(self, step, controls):
        if self.vehicle.flight_state.mode is not VehicleMode.VTOL:
            return self.vehicle.thrust_vector
        old_vector = self.vehicle.thrust_vector
        self.vehicle.thrust_vector = max(0.0, min(1.0, old_vector + controls.vector_command * VTOL.vector_rate * step))
        return (old_vector + self.vehicle.thrust_vector) / 2

    def _update_atmosphere(self, dt, controls):
        vehicle = self.vehicle
        if (vehicle.flight_state.status is FlightStatus.CRASHED or dt <= 0 or
                (vehicle.flight_state.status is FlightStatus.GROUNDED and
                 vehicle.flight_state.mode is VehicleMode.FIGHTER and not self.fighter_ground.on_runway())):
            return
        steps = max(1, math.ceil(dt / MAX_STEP))
        step = dt / steps
        for _ in range(steps):
            current_forces = calculate_forces(vehicle)
            vtol = vehicle.flight_state.mode is VehicleMode.VTOL
            if vtol:self.vtol_ground.begin(step)
            else:self.fighter_ground.begin()
            authority = 1.0 if vtol else control_authority(current_forces)
            pitch_rate = VTOL.pitch_rate_deg if vtol else PARAMETERS.pitch_authority_deg
            roll_rate = VTOL.roll_rate_deg if vtol else PARAMETERS.roll_authority_deg
            yaw_rate = VTOL.yaw_rate_deg if vtol else PARAMETERS.yaw_authority_deg
            rate_limits = dict(pitch=math.radians(pitch_rate)*authority,
                               roll=math.radians(roll_rate)*authority,
                               yaw=math.radians(yaw_rate)*authority)
            command = self.fcc.commands(controls, vehicle, step, rate_limits)
            self._apply_rates(command, rate_limits)
            rotation = dict(pitch=vehicle.pitch_rate * step / 2,
                            yaw=vehicle.yaw_rate * step / 2,
                            roll=vehicle.roll_rate * step / 2)
            old_throttle = vehicle.throttle
            vehicle.throttle = max(0.0, min(1.0, old_throttle + command.thrust * THROTTLE_RATE * step))
            throttle = (old_throttle + vehicle.throttle) / 2
            vector = self._update_vector(step, command)
            if command.throttle_override is not None:
                throttle = command.throttle_override
            vehicle.engine_throttle = throttle
            vehicle.rotate(**rotation)
            # Midpoint force integration: drag depends on velocity, not a decay percentage.
            initial_forces = calculate_forces(vehicle, throttle=throttle, controls=command, vector=vector)
            initial_acceleration=(self.vtol_ground.acceleration(initial_forces,vehicle.velocity,step)
                                  if vtol else self.fighter_ground.acceleration(initial_forces,vehicle.velocity,step))
            midpoint_velocity = vehicle.velocity + initial_acceleration * step / 2
            midpoint_forces = calculate_forces(vehicle, throttle=throttle, velocity=midpoint_velocity, controls=command, vector=vector)
            vehicle.acceleration = (self.vtol_ground.acceleration(midpoint_forces,midpoint_velocity,step)
                                    if vtol else self.fighter_ground.acceleration(midpoint_forces,midpoint_velocity,step))
            vehicle.position += midpoint_velocity * step
            vehicle.velocity += vehicle.acceleration * step
            vehicle.rotate(**rotation)
            if not np.isfinite(vehicle.position).all() or not np.isfinite(vehicle.velocity).all():
                raise ValueError("Atmospheric integration became nonfinite.")
            if vtol:
                self.vtol_ground.finish()
                if vehicle.flight_state.status is FlightStatus.CRASHED:break
            elif self.fighter_ground.on_runway():
                self.fighter_ground.finish()
                if vehicle.flight_state.status is FlightStatus.CRASHED:
                    self.fcc.clear_hover()
                    break
            elif vehicle.position[1] <= PARAMETERS.ground_altitude:
                impact_speed = vehicle.speed
                vehicle.position[1] = PARAMETERS.ground_altitude
                vehicle.flight_state.status = (FlightStatus.CRASHED if impact_speed >= PARAMETERS.crash_speed
                                               else FlightStatus.GROUNDED)
                vehicle.velocity[:] = 0
                vehicle.acceleration[:] = 0
                vehicle.pitch_rate = vehicle.yaw_rate = vehicle.roll_rate = 0.
                vehicle.aerodynamics = AeroForces()
                vehicle.engine_throttle = 0.
                self.fcc.clear_hover()
                break
        if (vehicle.flight_state.status is FlightStatus.FLYING or
                (vehicle.flight_state.status is FlightStatus.GROUNDED and
                 (vehicle.flight_state.mode is VehicleMode.VTOL or self.fighter_ground.on_runway()))):
            vehicle.aerodynamics = calculate_forces(vehicle, throttle=vehicle.engine_throttle, controls=command)
