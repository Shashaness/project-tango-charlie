"""Shared flat-ground penetration/normal-impulse resolution; no tangential reset."""
import math
import numpy as np
from game.flight_state import FlightStatus,VehicleMode


def resolve_contact(vehicle,clearance,*,supported,safe_speed,crash_speed,safe_tilt,
                    crash_tilt=180.,crash_horizontal_speed=math.inf,
                    release_epsilon=.03,hysteresis=False,hard_speed=math.inf):
    """Same zero-restitution contact used by Battledroid, configurable for VTOL."""
    v=vehicle;ground=v.ground_contact;ground.depth=0.
    if v.position[1]<=clearance:
        ground.depth=max(0.,clearance-v.position[1]);v.position[1]=clearance
        if v.velocity[1]<=0:
            impact=max(0.,-float(v.velocity[1]));horizontal=float(np.linalg.norm(v.velocity[[0,2]]))
            v.velocity[1]=0. # normal collision impulse only
            tilt=math.degrees(math.acos(float(np.clip(v.up[1],-1,1))))
            if not supported:
                ground.impact_speed=impact
                if impact>safe_speed or tilt>safe_tilt:
                    ground.hard_landing_time=5. if impact>=hard_speed else 3.
            crashed=(impact>crash_speed or tilt>crash_tilt or horizontal>crash_horizontal_speed)
            v.flight_state.status=FlightStatus.CRASHED if crashed else FlightStatus.GROUNDED
        elif not hysteresis:v.flight_state.status=FlightStatus.FLYING
    elif v.position[1]>clearance+release_epsilon or (not hysteresis and v.velocity[1]>0):
        v.flight_state.status=FlightStatus.FLYING


def transformation_clearance(vehicle):
    """Contact envelope for G/B deployment only; never reads M17 animated feet."""
    c=getattr(vehicle,'transformation',None)
    if c is None or not c.active or c._edge!=(VehicleMode.VTOL,VehicleMode.BATTLEDROID):return None
    from game.vtol_ground import foot_geometry
    from game.battledroid_physics import BATTLEDROID
    from game.transformation import phase
    u=phase(c._coordinate)
    return (1-u)*foot_geometry()[1]+u*BATTLEDROID.foot_clearance
