"""Canonical body skid support: no lift assistance or animated landing gear."""
from functools import lru_cache
import numpy as np
from game.atmospheric_physics import PARAMETERS
from game.flight_state import FlightStatus
from game.ground_contact import resolve_contact


@lru_cache(maxsize=1)
def fighter_vertices():
    from game.transformation import TransformationController
    c=TransformationController()
    points=[]
    for node in c.model.nodes:
        for index in node.meshes:
            p=c.model.primitives[index].positions
            points.extend((node.world_matrix@np.column_stack((p,np.ones(len(p)))).T).T[:,:3])
    points=np.asarray(points);points.setflags(write=False)
    return points


def clearance(vehicle):
    return -float((fighter_vertices()@vehicle.orientation.T)[:,1].min())+PARAMETERS.ground_altitude


class FighterGroundContact:
    def __init__(self,vehicle):
        self.vehicle=vehicle
        self.supported=False

    def on_runway(self):
        from game.world_environment import RUNWAY
        x,_,z = self.vehicle.position
        return abs(x-RUNWAY.center[0]) <= RUNWAY.width/2 and abs(z-RUNWAY.center[2]) <= RUNWAY.length/2

    def begin(self):
        v=self.vehicle;g=v.ground_contact
        g.reaction_force[:]=0;g.friction_force[:]=0
        self.supported=(self.on_runway() and v.position[1]<=clearance(v)+.001 and v.velocity[1]<=.001)

    def acceleration(self,forces,velocity,dt):
        force=forces.total_force.copy();g=self.vehicle.ground_contact
        if self.supported:
            load=max(0.,-force[1]);g.reaction_force[:]=(0,load,0)
            force[1]+=load
            tangent=np.asarray(velocity).copy();tangent[1]=0
            speed=np.linalg.norm(tangent)
            # Low rolling/skid resistance, bounded to avoid reversing motion.
            if speed>1e-8:
                friction=-tangent/speed*min(.015*load,PARAMETERS.mass*speed/dt)
                force+=friction;g.friction_force[:]=friction
        return force/PARAMETERS.mass

    def finish(self):
        v=self.vehicle
        resolve_contact(v,clearance(v),supported=self.supported,safe_speed=3.,
                        crash_speed=18.,safe_tilt=20.,crash_tilt=65.,release_epsilon=.02)
        v.ground_contact.clearance=clearance(v)-PARAMETERS.ground_altitude
