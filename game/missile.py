"""Finite rocket flight with separation, seeker, PN forces, burnout and expiry."""

from dataclasses import dataclass
from enum import Enum,auto
import math
import numpy as np

from engine.transform import Transform
from game.missile_guidance import proportional_navigation
from game.missile_seeker import MissileSeeker,SeekerState,SeekerType,select_candidate


@dataclass(frozen=True)
class MissileParameters:
    mass: float=100.0  # kg
    thrust: float=6000.0  # N
    burn_time: float=6.0  # seconds after separation
    separation_time: float=.2
    launch_speed: float=80.0  # m/s relative to launcher
    hardpoint: tuple=(1.2,-.5,-1.5)  # local right/up/backward meters
    max_lifetime: float=20.0
    max_travel: float=10000.0  # integrated path length, m
    navigation_constant: float=4.0
    max_g: float=35.0
    max_turn_rate_deg: float=90.0
    seeker_range: float=10000.0
    seeker_half_cone_deg: float=45.0
    seeker_loss_timeout: float=.8
    fuse_radius: float=20.0  # distance to target sphere surface
    min_range: float=200.0
    effective_range: float=4000.0
    max_range: float=8000.0
    nominal_engagement_speed: float=450.0  # envelope estimate, never assigned velocity
    lock_half_cone_deg: float=30.0
    lock_time: float=1.5
    max_inventory: int=12


MISSILE=MissileParameters()


class MotorState(Enum):
    SEPARATING=auto()
    BURNING=auto()
    COASTING=auto()


class Missile(Transform):
    def __init__(self,vehicle,target,parameters=MISSILE,seeker_type=SeekerType.RADAR):
        position=vehicle.position+vehicle.orientation@np.array(parameters.hardpoint)
        super().__init__(position)
        self.forward=vehicle.forward.copy();self.right=vehicle.right.copy();self.up=vehicle.up.copy()
        self.velocity=vehicle.velocity.copy()+vehicle.forward*parameters.launch_speed
        self.previous_position=self.position.copy()
        self.target=target  # compatibility alias for original launch assignment
        self.original_target=target
        self.current_seeker_target=target
        self.seeker_type=seeker_type
        self.parameters=parameters
        self.age=0.0;self.alive=True;self.travel=0.0
        self.motor_state=MotorState.SEPARATING
        self.seeker=MissileSeeker(parameters.seeker_range,parameters.seeker_half_cone_deg,parameters.seeker_loss_timeout)
        self.guidance_acceleration=np.zeros(3)
        self.acceleration=np.zeros(3)
        self.miss_reason=None

    @property
    def speed(self):
        return float(np.linalg.norm(self.velocity))

    @property
    def seeker_state(self):
        return self.seeker.state

    def _align_to_motion(self,dt):
        speed=self.speed
        if speed<1e-6:return
        direction=self.velocity/speed
        dot=float(np.clip(np.dot(self.forward,direction),-1,1))
        angle=math.acos(dot)
        if angle<1e-8:return
        axis=np.cross(self.forward,direction)
        length=float(np.linalg.norm(axis))
        if length<1e-8:axis=self.up.copy()
        else:axis/=length
        increment=min(angle,math.radians(self.parameters.max_turn_rate_deg)*dt)
        angular=axis*increment
        self.rotate(pitch=float(np.dot(angular,self.right)),yaw=float(np.dot(angular,self.up)),
                    roll=float(np.dot(angular,self.forward)))

    def update(self,dt,target_start=None,countermeasures=(),candidate_starts=None):
        if not math.isfinite(dt) or dt<0:raise ValueError('Missile timestep must be finite and nonnegative.')
        self.previous_position=self.position.copy()
        if not self.alive or dt==0:return 0.0
        if not self.target.alive and self.current_seeker_target is self.original_target:
            self.alive=False;self.miss_reason='TARGET DESTROYED';return 0.0
        origin=self.target.position if target_start is None else np.asarray(target_start)
        remaining=min(dt,max(0.0,self.parameters.max_lifetime-self.age))
        elapsed=0.0
        while elapsed<remaining-1e-12 and self.alive:
            step=min(1/120,remaining-elapsed)
            for boundary in (self.parameters.separation_time,self.parameters.separation_time+self.parameters.burn_time):
                if self.age<boundary<self.age+step:step=boundary-self.age
            separated=self.age>=self.parameters.separation_time-1e-10
            burning=separated and self.age<self.parameters.separation_time+self.parameters.burn_time-1e-10
            self.motor_state=MotorState.BURNING if burning else (MotorState.COASTING if separated else MotorState.SEPARATING)
            self.guidance_acceleration[:]=0
            if separated:
                positions = {self.original_target: origin+self.original_target.velocity*elapsed}
                for candidate in countermeasures:
                    start = candidate.position if candidate_starts is None else candidate_starts.get(candidate,candidate.position)
                    positions[candidate] = start+candidate.velocity*elapsed
                chosen = select_candidate(self,countermeasures,positions)
                if chosen is not None: self.current_seeker_target=chosen
                tracked = self.current_seeker_target
                tracked_position = positions.get(tracked,tracked.position)
                relative_position=tracked_position-self.position
                tracking=self.seeker.update(relative_position,self.forward,chosen is not None,step)
                if self.seeker.state is SeekerState.LOST:
                    self.alive=False;self.miss_reason='TRACK LOST';break
                if tracking:
                    self.guidance_acceleration=proportional_navigation(
                        relative_position,tracked.velocity-self.velocity,self.velocity,
                        self.parameters.navigation_constant,self.parameters.max_g,self.parameters.max_turn_rate_deg)
            thrust_acceleration=self.forward*self.parameters.thrust/self.parameters.mass if burning else np.zeros(3)
            self.acceleration=thrust_acceleration+self.guidance_acceleration
            displacement=self.velocity*step+self.acceleration*step*step/2
            self.position+=displacement
            self.velocity+=self.acceleration*step
            self.travel+=float(np.linalg.norm(displacement))
            if separated:self._align_to_motion(step)
            self.age+=step;elapsed+=step
            if not np.isfinite(self.position).all() or not np.isfinite(self.velocity).all():
                self.alive=False;self.miss_reason='INVALID STATE'
            elif self.travel>=self.parameters.max_travel:
                self.alive=False;self.miss_reason='MAX RANGE'
        if self.age>=self.parameters.max_lifetime-1e-10:
            self.alive=False;self.miss_reason='EXPIRED'
        if self.age>=self.parameters.separation_time+self.parameters.burn_time-1e-10:
            self.motor_state=MotorState.COASTING
        return elapsed
