"""Isolated Battledroid integration. Contact impulses are the only velocity constraints."""
import math
import numpy as np
from game.atmospheric_physics import PARAMETERS
from game.battledroid_physics import BATTLEDROID, forces, contact_clearance, walking_acceleration
from game.flight_state import Environment, FlightStatus

class BattledroidController:
    def __init__(self, vehicle, fcc):
        self.vehicle=vehicle;self.fcc=fcc
        self.jump_remaining=0.
        self._jump_down=False
        self.ground_height=lambda position:0.0  # replaceable flat-ground contact query

    def update(self, dt, pilot):
        if not math.isfinite(dt):raise ValueError('Delta time must be finite.')
        if dt<=0:return
        jump=bool(getattr(pilot,'jump_pressed',False))
        request=jump and not self._jump_down;self._jump_down=jump
        steps=max(1,math.ceil(dt/(1/120)));step=dt/steps
        for index in range(steps):self._step(step,pilot,request and index==0)

    def _step(self, dt, pilot, jump_request):
        v=self.vehicle;p=BATTLEDROID;ground=v.ground_contact
        ground.hard_landing_time=max(0.,ground.hard_landing_time-dt)
        if v.flight_state.status is FlightStatus.CRASHED:return
        atmosphere=v.flight_state.environment is Environment.ATMOSPHERE
        height=self.ground_height(v.position) if atmosphere else None
        # Contact is geometric, not merely a zero vertical velocity.
        supported=(atmosphere and height is not None and v.flight_state.status is FlightStatus.GROUNDED
                   and v.position[1]<=height+contact_clearance(v)+.03)
        if not supported and v.flight_state.status is FlightStatus.GROUNDED:
            v.flight_state.status=FlightStatus.FLYING
        if supported and jump_request:self.jump_remaining=p.jump_duration
        limits={axis:math.radians(getattr(p,axis+'_rate_deg')) for axis in ('pitch','yaw','roll')}
        command=self.fcc.commands(pilot,v,dt,limits)
        if supported:
            # Rotate body-up toward the support normal through bounded angular rates.
            axis=np.cross(v.up,ground.normal);angle=math.atan2(np.linalg.norm(axis),np.dot(v.up,ground.normal))
            if np.linalg.norm(axis)<1e-8:axis=v.right.copy() if angle>1 else np.zeros(3)
            else:axis/=np.linalg.norm(axis)
            angular=axis*min(p.stabilization_gain*angle,math.radians(p.stabilization_rate_deg))
            target_yaw=pilot.yaw*math.radians(p.ground_turn_rate_deg)
            change=np.clip(target_yaw-v.yaw_rate,-math.radians(p.ground_angular_acceleration_deg)*dt,math.radians(p.ground_angular_acceleration_deg)*dt)
            v.yaw_rate+=float(change)
            angular+=ground.normal*v.yaw_rate
            v.pitch_rate=float(np.dot(angular,v.right));v.roll_rate=float(np.dot(angular,v.forward))
            rotation=dict(pitch=v.pitch_rate*dt/2,yaw=float(np.dot(angular,v.up))*dt/2,roll=v.roll_rate*dt/2)
        else:
            for axis in limits:setattr(v,axis+'_rate',getattr(command,axis)*limits[axis])
            rotation={axis:getattr(v,axis+'_rate')*dt/2 for axis in limits}
        v.rotate(**rotation)
        old=v.throttle;v.throttle=float(np.clip(old+command.thrust*.5*dt,0,1))
        throttle=(old+v.throttle)/2
        if command.throttle_override is not None:throttle=command.throttle_override
        if self.jump_remaining>1e-10:
            throttle=1.;self.jump_remaining=max(0.,self.jump_remaining-dt)
        v.engine_throttle=throttle
        ground.reaction_force[:]=0;ground.friction_force[:]=0;ground.desired_velocity[:]=0
        initial=forces(v,command,throttle)
        acceleration=initial.total_force/PARAMETERS.mass
        if supported and acceleration[1]<=0:
            desired,walk=walking_acceleration(v,pilot,dt)
            ground.desired_velocity=desired
            # Contact supports gravity; walking/friction has a bounded tangential force.
            ground.reaction_force[1]=-initial.total_force[1]
            ground.friction_force=walk*PARAMETERS.mass
            acceleration=walk+initial.total_force/PARAMETERS.mass
            acceleration[1]=0
            midpoint=v.velocity+acceleration*dt/2
            v.position+=midpoint*dt;v.velocity+=acceleration*dt
        elif atmosphere:
            v.flight_state.status=FlightStatus.FLYING
            midpoint=v.velocity+acceleration*dt/2
            sampled=forces(v,command,throttle,midpoint)
            acceleration=sampled.total_force/PARAMETERS.mass
            v.position+=midpoint*dt;v.velocity+=acceleration*dt
        else:
            damping=p.space_damping+(p.space_brake_damping if command.brake else 0.)
            decay=math.exp(-damping*dt);integral=-math.expm1(-damping*dt)/damping
            v.position+=v.velocity*integral+acceleration*(dt-integral)/damping
            v.velocity=v.velocity*decay+acceleration*integral
        v.acceleration=acceleration
        v.rotate(**rotation)
        ground.depth=0.
        if atmosphere and height is not None:
            clearance=height+contact_clearance(v)
            from game.ground_contact import resolve_contact
            resolve_contact(v,clearance,supported=supported,
                safe_speed=p.safe_landing_vertical_speed,crash_speed=p.crash_vertical_speed,
                safe_tilt=p.safe_landing_tilt_deg)
            if v.flight_state.status is FlightStatus.CRASHED:self.fcc.clear_hover()
        if v.is_grounded:
            ground.walk_phase+=float(np.linalg.norm(v.velocity[[0,2]]))*dt*1.5
        v.aerodynamics=forces(v,command,throttle)
        if not np.isfinite(v.position).all() or not np.isfinite(v.velocity).all():raise ValueError('Nonfinite Battledroid physics.')
