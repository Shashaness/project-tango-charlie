"""VTOL foot geometry and contact tuning; approved airborne parameters unchanged."""
from dataclasses import dataclass
from functools import lru_cache
import numpy as np
from game.flight_state import VehicleMode

@dataclass(frozen=True)
class VTOLGroundParameters:
    contact_epsilon: float = .001
    liftoff_epsilon: float = .02
    landing_height: float = 2.
    safe_vertical_speed: float = 3.
    hard_vertical_speed: float = 7.
    crash_vertical_speed: float = 18.
    safe_tilt_deg: float = 25.
    crash_tilt_deg: float = 75.
    crash_horizontal_speed: float = 100.
    dynamic_friction: float = .12
    static_friction: float = .25
    friction_damping: float = .6
    static_speed: float = .03
    acceleration_epsilon: float = 1e-6
    body_clearance: float = .8

VTOL_GROUND=VTOLGroundParameters()

@lru_cache(maxsize=1)
def foot_geometry():
    """Immutable canonical foot mesh vertices, converted to vehicle coordinates."""
    from game.transformation import TransformationController
    c=TransformationController();c.reset(VehicleMode.VTOL)
    feet=[]
    for side in ('Left','Right'):
        points=[]
        def collect(node):
            for index in node.meshes:
                p=c.model.primitives[index].positions
                points.extend((node.world_matrix@np.column_stack((p,np.ones(len(p)))).T).T[:,:3])
            for child in node.children:collect(child)
        collect(c.model.find_node(side+'Foot'))
        values=np.asarray(points);values.setflags(write=False);feet.append(values)
    return tuple(feet),-min(p[:,1].min() for p in feet)

def contact_clearance(vehicle):
    from game.ground_contact import transformation_clearance
    transition=transformation_clearance(vehicle)
    if transition is not None:return transition
    feet,_=foot_geometry()
    return max(VTOL_GROUND.body_clearance,-min((p@vehicle.orientation.T)[:,1].min() for p in feet))
