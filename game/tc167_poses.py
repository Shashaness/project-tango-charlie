"""M16 rest-relative visual poses, source GLB coordinates (+Z nose, +Y up).

Joint translations below are absolute parent-local values. Only changed nodes
are listed; the approved binary and imported rest data are never modified.
"""
from dataclasses import dataclass
import math
from engine.quaternion import multiply
from game.flight_state import VehicleMode

@dataclass(frozen=True)
class PoseTransform:
    translation: tuple
    rotation: tuple = (0.,0.,0.,1.)
    scale: tuple = (1.,1.,1.)


def rotation(x=0,y=0,z=0):
    def axis(index,degrees):
        q=[0.,0.,0.,math.cos(math.radians(degrees)/2)]
        q[index]=math.sin(math.radians(degrees)/2)
        return q
    return tuple(multiply(multiply(axis(0,x),axis(1,y)),axis(2,z)))

FIGHTER_POSE = {}
VTOL_POSE = {}
BATTLEDROID_POSE = {
    # Visual-only root datum: native-length legs keep soles at physical -3.2 m.
    'TC167Root': PoseTransform((0,3.2873,0)),
    'Fuselage': PoseTransform((0,1.3,0),rotation(x=-90)),
    'FuselageSurface1': PoseTransform((0,0,0),scale=(.85,.85,.38)),
    'Nose': PoseTransform((0,-1.4,1),rotation(x=180),(.8,.8,.8)),
    'Cockpit': PoseTransform((0,-.8,.3),rotation(x=90)),
    'TorsoCore': PoseTransform((0,0,0)),
    'Head': PoseTransform((0,-.05,2.6)),
    'VerticalTail': PoseTransform((0,1.8,-1.8),rotation(x=-90)),
}
for side,label in ((1,'Left'),(-1,'Right')):
    VTOL_POSE.update({
        label+'Intake': PoseTransform((side*1.6,-.1,2),rotation(x=-8)),
        label+'UpperLeg': PoseTransform((0,0,-1.5),rotation(x=-57)),
        label+'LowerLeg': PoseTransform((0,0,-2.6),rotation(x=-35)),
        label+'Foot': PoseTransform((0,0,-2.6),rotation(x=-80)),
        label+'Wing': PoseTransform((side*1.3,0,.5),rotation(z=side*8)),
        label+'Tail': PoseTransform((side*.8,.3,-6),rotation(x=8)),
    })
    BATTLEDROID_POSE.update({
        label+'Intake': PoseTransform((side*1.1,-.6,-.4),scale=(.85,1.,1.)),
        label+'UpperLeg': PoseTransform((0,0,0),rotation(x=-94)),
        label+'LowerLeg': PoseTransform((0,0,-2.6),rotation(x=8)),
        label+'Foot': PoseTransform((0,0,-2.6),rotation(x=-94)),
        label+'Wing': PoseTransform((side*1.25,2.8,-1.5+side*.1),rotation(x=-90,y=side*90)),
        label+'Tail': PoseTransform((side*.8,1,-2),rotation(x=-75,y=side*65)),
        label+'Shoulder': PoseTransform((side*1.5,2.,-.2)),
        label+'UpperArm': PoseTransform((0,0,0),rotation(x=-90,y=-side*12),(.9,.9,.75)),
        label+'Forearm': PoseTransform((0,0,-2),rotation(x=-20)),
        label+'Hand': PoseTransform((0,0,-1.8),rotation()),
    })
# Retain M16.2 intake-shell proportions without compressing their leg children.
for name in ('LeftIntakeSurface8','LeftIntakeSurface9','LeftIntakeSurface10',
             'RightIntakeSurface21','RightIntakeSurface22','RightIntakeSurface23'):
    BATTLEDROID_POSE[name] = PoseTransform((0,0,0),scale=(1.,.448,1.))

POSES = {VehicleMode.FIGHTER:FIGHTER_POSE,VehicleMode.VTOL:VTOL_POSE,
         VehicleMode.BATTLEDROID:BATTLEDROID_POSE}
# Engine collars remain attached to each foot throughout deployment; these
# additional local rotations turn the nozzles downward in the deployed modes.
ENGINE_ROTATIONS = {VehicleMode.FIGHTER:rotation(),VehicleMode.VTOL:rotation(x=90),
                    VehicleMode.BATTLEDROID:rotation(x=90)}

ENGINE_OFFSETS = {VehicleMode.FIGHTER:(0.,0.,0.),
                  VehicleMode.VTOL:(0.,1.35,-1.7),
                  VehicleMode.BATTLEDROID:(0.,1.46,-1.7)}

# Timing on the forward edges. Reversal evaluates the exact same curve backward.
TIMINGS = {
    'Intake':(0,.25), 'UpperLeg':(.1,.5), 'LowerLeg':(.25,.7),
    'Foot':(.45,.85), 'Engine':(.45,.85), 'Wing':(.6,1), 'Tail':(.6,1),
}
BATTLEDROID_TIMINGS = {
    'Intake':(.3,.9), 'UpperLeg':(.35,.8), 'LowerLeg':(.4,.85), 'Foot':(.55,.95),
    'TC167Root':(.35,.9),
    'Fuselage':(.1,.6), 'Nose':(.15,.65), 'Cockpit':(.2,.65), 'TorsoCore':(.1,.6),
    'Shoulder':(.35,.7), 'UpperArm':(.4,.8), 'Forearm':(.5,.9), 'Hand':(.6,.95),
    'Head':(.75,1), 'Wing':(.45,.95), 'Tail':(.3,.85), 'Engine':(.2,.6),
}
DURATIONS = {(VehicleMode.FIGHTER,VehicleMode.VTOL):1.5,
             (VehicleMode.VTOL,VehicleMode.BATTLEDROID):1.9}
PHYSICS_SWITCH = .6
