"""Read-only physical-state sampling and local visual offsets over M16 poses."""
import math
import numpy as np
from engine.model import trs
from game.flight_state import Environment, FlightStatus, VehicleMode
from game.tc167_poses import rotation

IDLE_SPEED_THRESHOLD=.3
RUN_SPEED_THRESHOLD=4.5
RUN_BLEND_START=3.5
RUN_BLEND_END=6.
WALK_MIN_CADENCE=.35  # complete left/right cycles per second
WALK_MAX_CADENCE=.75
RUN_MAX_CADENCE=1.05
WALK_HIP_SWING=20.
RUN_HIP_SWING=28.
STRAFE_HIP_SWING=6.  # shared lateral weight shift; avoid an inward scissor stance
WALK_KNEE_FLEX=25.
RUN_KNEE_FLEX=38.
WALK_ARM_SWING=12.
RUN_ARM_SWING=18.
ANKLE_AMPLITUDE=6.
TORSO_AMPLITUDE=1.5
ANIMATION_BLEND_RATE=5.  # amplitude units/s: 0.2 s full ramp
TURN_RATE_THRESHOLD=math.radians(3)
TURN_CADENCE=.4
TURN_HIP_SWING=7.


def approach(value,target,amount):
    return value+float(np.clip(target-value,-amount,amount))

class BattledroidLocomotionAnimator:
    """Owns only animation state and model deltas, never vehicle integration."""
    def __init__(self,transformation,enabled=True):
        self.transformation=transformation;self.model=transformation.model
        self.enabled=enabled;self.state='IDLE';self.gait_phase=0.;self.blend=0.
        self.forward_speed=self.lateral_speed=self.cadence=0.
        self.offsets={};self.joint_angles={};self._fading_angles={}
        self._was_transforming=False
        self.names=tuple(side+part for side in ('Left','Right')
                         for part in ('UpperLeg','LowerLeg','Foot','UpperArm'))+('TorsoCore',)
        self.nodes={name:self.model.find_node(name) for name in self.names}
        self.engines={side:self.model.find_node(side+'Engine') for side in ('Left','Right')}

    def clear_layer(self):
        for node in (*self.nodes.values(),*self.engines.values()):node.set_animation_delta()

    def update(self,dt,vehicle,pilot):
        if not math.isfinite(dt) or dt<0:raise ValueError('Animation dt must be finite and nonnegative')
        self.clear_layer()
        c=self.transformation
        # Horizontal body heading matches M13 locomotion, independent of tilt.
        forward=vehicle.forward.copy();forward[1]=0
        length=np.linalg.norm(forward)
        forward=forward/length if length>1e-8 else np.array((0.,0.,-1.))
        right=np.cross(forward,(0.,1.,0.))
        horizontal=vehicle.velocity.copy();horizontal[1]=0
        speed=float(np.linalg.norm(horizontal))
        self.forward_speed=float(np.dot(horizontal,forward))
        self.lateral_speed=float(np.dot(horizontal,right))
        eligible=(self.enabled and not c.active and c.configuration is VehicleMode.BATTLEDROID
                  and vehicle.flight_state.mode is VehicleMode.BATTLEDROID
                  and vehicle.flight_state.environment is Environment.ATMOSPHERE
                  and vehicle.flight_state.status is FlightStatus.GROUNDED)
        moving=abs(getattr(pilot,'pitch',0))+abs(getattr(pilot,'roll',0))>1e-8
        turning=abs(vehicle.yaw_rate)>TURN_RATE_THRESHOLD and speed<IDLE_SPEED_THRESHOLD
        if eligible and speed>=IDLE_SPEED_THRESHOLD:
            self.state=('RUN' if speed>RUN_SPEED_THRESHOLD else 'WALK') if moving else 'SKID'
        elif eligible and turning:self.state='TURN'
        else:self.state='IDLE'
        active=self.state in ('WALK','RUN','TURN')
        # Completion frame exposes the exact M16 endpoint before ramping in.
        if self._was_transforming and not c.active:
            self.blend=0.;active=False;self._fading_angles={}
        self._was_transforming=c.active
        self.blend=approach(self.blend,1. if active else 0.,ANIMATION_BLEND_RATE*dt)
        if not self.enabled or c.configuration is not VehicleMode.BATTLEDROID:
            self.blend=0.
        if active:
            run=float(np.clip((speed-RUN_BLEND_START)/(RUN_BLEND_END-RUN_BLEND_START),0,1))
            self.cadence=(WALK_MIN_CADENCE+(WALK_MAX_CADENCE-WALK_MIN_CADENCE)*min(speed/RUN_SPEED_THRESHOLD,1))
            self.cadence+=(RUN_MAX_CADENCE-WALK_MAX_CADENCE)*run
            if self.state=='TURN':self.cadence=TURN_CADENCE
            self.gait_phase=(self.gait_phase+math.tau*self.cadence*dt)%math.tau
            f=self.forward_speed/max(speed,1e-8);l=self.lateral_speed/max(speed,1e-8)
            hip=WALK_HIP_SWING+(RUN_HIP_SWING-WALK_HIP_SWING)*run
            knee=WALK_KNEE_FLEX+(RUN_KNEE_FLEX-WALK_KNEE_FLEX)*run
            arm=WALK_ARM_SWING+(RUN_ARM_SWING-WALK_ARM_SWING)*run
            if self.state=='TURN':
                f=math.copysign(1.,vehicle.yaw_rate);l=0.;hip=TURN_HIP_SWING;knee=10.;arm=3.
            angles={}
            for index,side in enumerate(('Left','Right')):
                p=self.gait_phase+index*math.pi
                swing=math.sin(p);recovery=max(0.,math.cos(p))**2
                # Lateral weight shift is shared; recovery knees still alternate.
                angles[side+'UpperLeg']=(-hip*swing*f,STRAFE_HIP_SWING*math.sin(self.gait_phase)*l,0.)
                angles[side+'LowerLeg']=(knee*recovery,0.,0.)
                angles[side+'Foot']=(-ANKLE_AMPLITUDE*math.sin(p)*abs(f),0.,0.)
                angles[side+'UpperArm']=(arm*swing*f,0.,-arm*.35*swing*l)
            angles['TorsoCore']=(0.,TORSO_AMPLITUDE*math.sin(self.gait_phase)*f,0.)
            self._fading_angles=angles
        else:self.cadence=0. # freeze phase; fade the last offsets, including during takeoff/transform
        self.offsets={};self.joint_angles={}
        if self.blend==0:
            self._fading_angles={};self.model.world_matrices();return
        # clear_layer may have changed caches: canonical matrices are authoritative.
        self.model.world_matrices()
        old_feet={side:self.model.find_node(side+'Foot').world_matrix.copy() for side in self.engines}
        old_engines={side:node.world_matrix.copy() for side,node in self.engines.items()}
        for name,angles in self._fading_angles.items():
            angles=tuple(a*self.blend for a in angles);self.joint_angles[name]=angles
            delta=trs(rotation=rotation(*angles));self.offsets[name]=delta
            self.nodes[name].set_animation_delta(delta)
        self.model.world_matrices()
        # Engines are root siblings, not ankle children: follow the ankle delta once.
        for side,engine in self.engines.items():
            foot=self.model.find_node(side+'Foot').world_matrix
            desired=np.linalg.inv(engine.parent.world_matrix)@foot@np.linalg.inv(old_feet[side])@old_engines[side]
            engine.set_animation_delta(np.linalg.inv(engine.local_matrix)@desired)
        self.model.world_matrices()
