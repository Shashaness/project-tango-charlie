"""Bounded pilot-command augmentation; never writes vehicle state."""

from dataclasses import dataclass
import math

import numpy as np

from game.atmospheric_physics import PARAMETERS, calculate_forces
from game.flight_state import Environment, VehicleMode, FlightStatus
from game.vtol_physics import VTOL, thrust_direction

SAS_RATE_DAMPING = 8.0  # exponential rate decay /s on released axes
HOVER_VERTICAL_DAMPING = 1.5  # vertical velocity feedback /s
HOVER_OVERRIDE_SECONDS = 0.75  # pilot owns thrust during input and briefly afterward
MIN_UPWARD_THRUST_COMPONENT = 0.15  # don't demand hover with nearly horizontal thrust


@dataclass
class ControlCommands:
    pitch: float = 0.0
    yaw: float = 0.0
    roll: float = 0.0
    thrust: float = 0.0  # manual throttle rate command
    lift: float = 0.0
    strafe: float = 0.0
    brake: bool = False
    vector_command: float = 0.0
    throttle_override: float | None = None  # absolute engine command, not pilot setting


class FlightControlComputer:
    def __init__(self):
        self.stability_assist_enabled = True
        self.hover_assist_enabled = False
        self.hover_target_altitude = None
        self.hover_correction_active = False
        self._override_remaining = 0.0

    @staticmethod
    def hover_eligible(vehicle):
        state = vehicle.flight_state
        return (state.environment is Environment.ATMOSPHERE
                and state.mode in (VehicleMode.VTOL,VehicleMode.BATTLEDROID)
                and (state.status is FlightStatus.FLYING or
                     (state.mode is VehicleMode.VTOL and state.status is FlightStatus.GROUNDED)))

    def toggle_stability(self):
        self.stability_assist_enabled = not self.stability_assist_enabled

    def clear_hover(self):
        self.hover_assist_enabled = False
        self.hover_target_altitude = None
        self.hover_correction_active = False
        self._override_remaining = 0.0

    def toggle_hover(self, vehicle):
        if self.hover_assist_enabled:
            self.clear_hover()
        elif self.hover_eligible(vehicle):
            self.hover_assist_enabled = True
            self.hover_target_altitude = float(vehicle.position[1])
            self._override_remaining = 0.0

    def commands(self, pilot, vehicle, dt, rate_limits):
        """Read feedback and return controls; attitude targets are never generated."""
        if not math.isfinite(dt) or dt < 0:
            raise ValueError("FCC timestep must be finite and nonnegative.")
        if vehicle.flight_state.mode is VehicleMode.BATTLEDROID:
            return self._battledroid_commands(pilot,vehicle,dt,rate_limits)
        command = ControlCommands(**{name: getattr(pilot, name) for name in
                                  ('pitch','yaw','roll','thrust','lift','strafe','brake','vector_command')})
        state = vehicle.flight_state
        sas_applicable = (state.mode is VehicleMode.VTOL or
                          state.environment is Environment.ATMOSPHERE)
        if self.stability_assist_enabled and sas_applicable:
            for axis in ('pitch', 'yaw', 'roll'):
                if abs(getattr(command, axis)) < 1e-8:
                    limit = rate_limits[axis]
                    # Only rate feedback: a banked/inverted attitude is not an error.
                    rate = getattr(vehicle, axis + '_rate') * math.exp(-SAS_RATE_DAMPING * dt)
                    value = rate / limit if limit > 1e-8 else 0.0
                    setattr(command, axis, float(np.clip(value, -1, 1)))
        self.hover_correction_active = False
        if not self.hover_eligible(vehicle):
            self.clear_hover()
            return command
        if not self.hover_assist_enabled:
            return command
        if any(abs(value) > 1e-8 for value in (pilot.thrust, pilot.vector_command, pilot.lift)):
            self._override_remaining = HOVER_OVERRIDE_SECONDS
            return command
        self._override_remaining = max(0.0, self._override_remaining - dt)
        if self._override_remaining > 0:
            return command
        upward_component = float(thrust_direction(vehicle)[1])
        if upward_component < MIN_UPWARD_THRUST_COMPONENT:
            return command  # impossible attitude/vector: leave control with the pilot
        # Include gravity, aero forces and current maneuvering thrust, but no engine.
        passive_forces = calculate_forces(vehicle, throttle=0.0, controls=command)
        desired_acceleration = -float(vehicle.velocity[1]) * HOVER_VERTICAL_DAMPING
        required_engine_force = PARAMETERS.mass * desired_acceleration - passive_forces.total_force[1]
        available_upward_force = VTOL.max_thrust * upward_component
        command.throttle_override = float(np.clip(required_engine_force / available_upward_force, 0, 1))
        self.hover_correction_active = True
        return command

    def _battledroid_commands(self, pilot, vehicle, dt, rate_limits):
        """Battledroid-only extension; established VTOL hover path stays unchanged."""
        from game.battledroid_physics import BATTLEDROID, forces
        command=ControlCommands(**{name:getattr(pilot,name) for name in
            ('pitch','yaw','roll','thrust','lift','strafe','brake','vector_command')})
        if self.stability_assist_enabled and not vehicle.is_grounded:
            for axis in ('pitch','yaw','roll'):
                if abs(getattr(command,axis))<1e-8:
                    rate=getattr(vehicle,axis+'_rate')*math.exp(-SAS_RATE_DAMPING*dt)
                    setattr(command,axis,float(np.clip(rate/rate_limits[axis],-1,1)))
        self.hover_correction_active=False
        if not self.hover_eligible(vehicle):
            self.clear_hover();return command
        if not self.hover_assist_enabled:return command
        if abs(pilot.thrust)+abs(pilot.lift)>1e-8 or getattr(pilot,'jump_pressed',False):
            self._override_remaining=HOVER_OVERRIDE_SECONDS;return command
        self._override_remaining=max(0.,self._override_remaining-dt)
        if self._override_remaining>0:return command
        upward=float(vehicle.up[1])
        if upward<MIN_UPWARD_THRUST_COMPONENT:return command
        passive=forces(vehicle,command,throttle=0.)
        required=PARAMETERS.mass*(-vehicle.velocity[1]*HOVER_VERTICAL_DAMPING)-passive.total_force[1]
        available=PARAMETERS.mass*PARAMETERS.gravity*BATTLEDROID.thrust_to_weight*upward
        command.throttle_override=float(np.clip(required/available,0,1))
        self.hover_correction_active=True
        return command
