"""Contact force modifier for the existing VTOL integrator; no flight retuning."""
import numpy as np
from game.atmospheric_physics import PARAMETERS
from game.vtol_ground import VTOL_GROUND as P,contact_clearance,foot_geometry
from game.ground_contact import resolve_contact
from game.flight_state import FlightStatus

class VTOLGroundContact:
    def __init__(self,vehicle,fcc):
        self.vehicle=vehicle;self.fcc=fcc
        self.ground_height=lambda position:PARAMETERS.ground_altitude
        self.supported=False;self.clearance=0.;self.height=None

    def begin(self,dt):
        v=self.vehicle;g=v.ground_contact
        g.hard_landing_time=max(0.,g.hard_landing_time-dt)
        g.reaction_force[:]=0;g.friction_force[:]=0;g.depth=0
        self.height=self.ground_height(v.position)
        self.clearance=contact_clearance(v)+(self.height or 0.)
        if self.height is None and v.flight_state.status is FlightStatus.GROUNDED:
            v.flight_state.status=FlightStatus.FLYING
        self.supported=(self.height is not None and v.flight_state.status is FlightStatus.GROUNDED
                        and v.position[1]<=self.clearance+P.contact_epsilon and v.velocity[1]<=.001)

    def acceleration(self,forces,velocity,dt):
        """Add unilateral normal support and load-dependent tangential friction."""
        v=self.vehicle;g=v.ground_contact;force=forces.total_force.copy()
        if not self.supported:return force/PARAMETERS.mass
        load=max(0.,-float(force[1]))
        g.reaction_force[:]=(0.,load,0.)
        if force[1]<=P.acceleration_epsilon*PARAMETERS.mass:force[1]=0.
        horizontal=np.asarray(velocity).copy();horizontal[1]=0.
        speed=float(np.linalg.norm(horizontal));friction=np.zeros(3)
        if load>0:
            tangential=force.copy();tangential[1]=0
            if speed<P.static_speed:
                needed=-tangential-horizontal*PARAMETERS.mass/dt
                magnitude=float(np.linalg.norm(needed))
                friction=needed*min(1.,P.static_friction*load/max(magnitude,1e-12))
            elif speed>0:
                magnitude=min(P.dynamic_friction*load,PARAMETERS.mass*P.friction_damping*speed,
                              PARAMETERS.mass*speed/dt)
                friction=-horizontal/speed*magnitude
        g.friction_force[:]=friction
        return (force+friction)/PARAMETERS.mass

    def finish(self):
        v=self.vehicle;g=v.ground_contact
        if self.height is not None:
            self.clearance=contact_clearance(v)+self.height
            resolve_contact(v,self.clearance,supported=self.supported,
                safe_speed=P.safe_vertical_speed,crash_speed=P.crash_vertical_speed,
                safe_tilt=P.safe_tilt_deg,crash_tilt=P.crash_tilt_deg,
                crash_horizontal_speed=P.crash_horizontal_speed,
                release_epsilon=P.liftoff_epsilon,hysteresis=True,hard_speed=P.hard_vertical_speed)
            if v.flight_state.status is FlightStatus.CRASHED:self.fcc.clear_hover()
        feet,_=foot_geometry()
        heights=[float((p@v.orientation.T)[:,1].min()+v.position[1]-(self.height or 0.)) for p in feet]
        g.left_foot_height,g.right_foot_height=heights
        g.clearance=self.clearance-(self.height or 0.)
        g.landing_approach=(v.flight_state.status is FlightStatus.FLYING and self.height is not None
                            and 0<v.position[1]-self.clearance<P.landing_height and v.velocity[1]<0)
