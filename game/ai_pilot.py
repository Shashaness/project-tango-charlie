"""Deterministic pilot decisions. No writes to aircraft transforms or momentum."""
from dataclasses import dataclass
from enum import Enum, auto
import math
import numpy as np

from game.ai_config import AI
from game.atmospheric_physics import PARAMETERS, calculate_forces
from game.flight_control_computer import ControlCommands
from game.flight_state import Environment
from game.fire_control_solution import solve_intercept
from game.missile_fire_control import LockState

class AIState(Enum):
    PATROL = auto()
    PURSUIT = auto()
    ATTACK = auto()
    EVADE = auto()
    RECOVER = auto()
    DEAD = auto()

@dataclass
class AIPilotInput(ControlCommands):
    throttle: float = 0.0  # desired persistent throttle; adapted to thrust rate
    fire_gun: bool = False
    fire_missile: bool = False


def unit(vector, fallback):
    length = float(np.linalg.norm(vector))
    return vector / length if length > 1e-8 else np.asarray(fallback).copy()

class AIPilot:
    def __init__(self, parameters=AI, seed=0):
        self.parameters = parameters
        self.seed = seed
        self.state = AIState.PATROL
        self.elapsed = 0.0
        self.desired_direction = np.array((0., 0., -1.))
        self.lead_direction = self.desired_direction.copy()
        self.commands = AIPilotInput()
        self.missile_threat = False
        self.terrain_danger = False
        self.range_to_player = 0.0
        self._burst_phase = seed * .17
        self._missile_wait = seed * .4

    def pursuit(self, aircraft, target):
        relative = target.position - aircraft.position
        pure = unit(relative, aircraft.forward)
        solution = solve_intercept(aircraft.position, aircraft.velocity,
                                   target.position, target.velocity,
                                   max(aircraft.speed, self.parameters.target_speed))
        self.lead_direction = pure if solution is None else solution.required_direction
        return pure if np.linalg.norm(relative) < self.parameters.pure_pursuit_range else self.lead_direction.copy()

    def update(self, dt, aircraft, player, radar, missile_control, incoming=(), player_firing=False):
        if not math.isfinite(dt) or dt < 0:
            raise ValueError('AI timestep must be finite and nonnegative.')
        p = self.parameters
        self.elapsed += dt
        self._missile_wait = max(0., self._missile_wait-dt)
        command = AIPilotInput()
        if not aircraft.alive:
            self.state = AIState.DEAD
            self.commands = command
            return command
        relative = player.position-aircraft.position
        self.range_to_player = float(np.linalg.norm(relative))
        los = unit(relative, aircraft.forward)
        atmosphere = aircraft.flight_state.environment is Environment.ATMOSPHERE
        forces = calculate_forces(aircraft) if atmosphere else aircraft.aerodynamics
        alpha = forces.alpha
        self.missile_threat = bool(aircraft.avionics.incoming) if hasattr(aircraft,"avionics") and aircraft.avionics.incoming else any(m.alive and m.target is aircraft for m in incoming)
        behind = float(np.dot(los, aircraft.forward)) < -.35
        aimed = float(np.dot(-los, player.forward)) > .94
        gun_threat = self.range_to_player < 1800 and aimed and (behind or player_firing)
        self.terrain_danger = atmosphere and (aircraft.position[1] < p.min_altitude or
            aircraft.position[1] + min(0., aircraft.velocity[1])*p.terrain_lookahead < p.min_altitude)
        stalled = atmosphere and (abs(alpha) >= math.radians(PARAMETERS.stall_angle_deg) or aircraft.speed < p.min_combat_speed*.65)
        recovering = self.state is AIState.RECOVER and atmosphere and (
            aircraft.speed < p.min_combat_speed or abs(alpha) > math.radians(10))
        if stalled or recovering or self.terrain_danger:
            self.state = AIState.RECOVER
        elif self.missile_threat or gun_threat:
            self.state = AIState.EVADE
        elif self.range_to_player > p.detection_range or not player.alive:
            self.state = AIState.PATROL
        elif radar.shoot:
            self.state = AIState.ATTACK
        else:
            self.state = AIState.PURSUIT
        desired = self.pursuit(aircraft, player)
        if self.state is AIState.ATTACK and radar.solution is not None:
            desired = radar.solution.required_direction.copy()
        if self.state is AIState.PATROL:
            point = np.array((math.sin(self.seed+ self.elapsed*.025)*2500, 1000., -2500.))
            desired = unit(point-aircraft.position, aircraft.forward)
        elif self.state is AIState.EVADE:
            sign = 1 if (int(self.elapsed/p.break_period)+self.seed)%2 else -1
            desired = unit(aircraft.forward*.35+aircraft.right*sign+aircraft.up*.3, aircraft.forward)
        if self.terrain_danger:
            horizontal = aircraft.forward.copy(); horizontal[1] = 0
            desired = unit(unit(horizontal, (0,0,-1))+np.array((0,.7,0)), aircraft.up)
        self.desired_direction = desired.copy()
        throttle = float(np.clip(.22+(p.target_speed-aircraft.speed)*.012, 0, 1)) if atmosphere else .8
        if aircraft.speed > p.high_speed: throttle = 0.
        if self.state in (AIState.RECOVER, AIState.EVADE): throttle = 1.
        command.throttle = throttle
        # Adapter preserves the normal controller's gradual persistent throttle.
        command.thrust = float(np.clip((throttle-aircraft.throttle)/max(.01,p.reaction_time), -1, 1))
        x = float(np.dot(desired, aircraft.right))
        y = float(np.dot(desired, aircraft.up))
        z = float(np.dot(desired, aircraft.forward))
        if atmosphere:
            # Roll puts local up into the desired turn plane. Behind targets require
            # a sustained reversal through the same controls, never a snapped heading.
            plane = desired-aircraft.forward*z
            if np.linalg.norm(plane) < .02:
                plane = np.array((0.,1.,0.))-aircraft.forward*aircraft.forward[1]
            bank_error = math.atan2(float(np.dot(plane, aircraft.right)), float(np.dot(plane, aircraft.up)))
            command.roll = p.aggression*bank_error
            turn_angle = math.acos(float(np.clip(z,-1,1)))
            q_area = max(1., forces.dynamic_pressure*PARAMETERS.wing_area)
            trim_alpha = PARAMETERS.mass*PARAMETERS.gravity/(q_area*PARAMETERS.lift_curve_slope)
            desired_alpha = min(math.radians(14), trim_alpha+turn_angle*.22*max(0.,math.cos(bank_error)))
            command.pitch = (desired_alpha-alpha)*3.5
            command.yaw = -forces.beta*.4  # sideslip correction, not turret steering
            if self.state is AIState.RECOVER and not self.terrain_danger:
                command.pitch = (math.radians(3)-alpha)*3.5
                command.roll = 0.; command.yaw = 0.
            if self.terrain_danger:
                command.pitch = (math.radians(17)*max(0.,math.cos(bank_error))-alpha)*3.5
            if abs(alpha) > math.radians(PARAMETERS.stall_angle_deg*PARAMETERS.high_aoa_warning_fraction):
                command.pitch = -alpha*2.5  # unload even during dangerous terrain geometry
        else:
            command.pitch = p.aggression*math.atan2(y, max(.01,z))
            command.yaw = -p.aggression*math.atan2(x,z)
            command.roll = 0.
        for axis in ('pitch','yaw','roll'):
            setattr(command, axis, float(np.clip(getattr(command,axis),-1,1)))
        self._burst_phase = (self._burst_phase+dt)%(p.burst_duration+p.burst_pause)
        command.fire_gun = (p.guns_enabled and self.state is AIState.ATTACK and radar.shoot
                            and self._burst_phase < p.burst_duration)
        command.fire_missile = (p.missiles_enabled and self.state in (AIState.PURSUIT,AIState.ATTACK)
            and self._missile_wait == 0 and missile_control.selected_inventory > 0
            and missile_control.lock_state is LockState.LOCKED and missile_control.in_envelope)
        self.commands = command
        return command

    def missile_launched(self):
        self._missile_wait = self.parameters.missile_cooldown
