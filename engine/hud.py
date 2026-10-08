"""Read-only flight instruments drawn in framebuffer pixels with core OpenGL."""

import math

import numpy as np
from OpenGL import GL

from engine.mesh import Mesh
from game.flight_state import Environment, FlightStatus, VehicleMode
from game.atmospheric_physics import PARAMETERS
from game.missile_fire_control import LockState
from game.missile_seeker import SeekerType
from game.defensive_avionics import ThreatState
from game.flight_instruments import heading_degrees, vertical_speed, normal_g_load, attitude_angles

MIN_MARKER_SPEED = 0.05
HUD_COLOR = (0.3, 1.0, 0.55)
VELOCITY_COLOR = (1.0, 0.8, 0.2)
# Tiny 3x5 debug font: only instrumentation labels, numbers and punctuation.
FONT = dict(zip(
    '0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ.% -',
    ('111/101/101/101/111', '010/110/010/010/111', '111/001/111/100/111',
     '111/001/111/001/111', '101/101/111/001/001', '111/100/111/001/111',
     '111/100/111/101/111', '111/001/010/010/010', '111/101/111/101/111',
     '111/101/111/001/111',
     '010/101/111/101/101', '110/101/110/101/110', '111/100/100/100/111',
     '110/101/101/101/110', '111/100/110/100/111', '111/100/110/100/100',
     '111/100/101/101/111', '101/101/111/101/101', '111/010/010/010/111',
     '001/001/001/101/111', '101/101/110/101/101', '100/100/100/100/111',
     '101/111/111/101/101', '101/111/111/111/101', '111/101/101/101/111',
     '111/101/111/100/100', '111/101/101/111/001', '110/101/110/101/101',
     '111/100/111/001/111', '111/010/010/010/010', '101/101/101/101/111',
     '101/101/101/101/010', '101/101/111/111/101', '101/101/010/101/101',
     '101/101/010/010/010', '111/001/010/100/111',
     '000/000/000/000/010', '101/001/010/100/101',
     '000/000/000/000/000', '000/000/111/000/000')
))

FONT["/"] = "001/001/010/100/100"
FONT["+"] = "000/010/111/010/000"
FONT[">"] = "100/010/001/010/100"


def project_direction(direction, camera, width, height):
    """Project a world direction at infinity (w=0); return pixel point, edge flag.

    Translation is intentionally excluded: this indicates velocity direction,
    not an arbitrary finite point ahead of the vehicle. Rear/offscreen travel
    is represented by a perimeter chevron, never a false on-screen circle.
    """
    direction = np.asarray(direction, dtype=float)
    if direction.shape != (3,) or not np.isfinite(direction).all():
        return None
    length = np.linalg.norm(direction)
    if not math.isfinite(length) or length < MIN_MARKER_SPEED or width <= 0 or height <= 0:
        return None
    clip = camera.projection_matrix() @ camera.view_matrix() @ np.array((* (direction / length), 0.0))
    if not np.isfinite(clip).all():
        return None
    front = clip[3] > 1e-6
    ndc = clip[:2] / clip[3] if front else clip[:2]
    margin_x, margin_y = min(24 / width, .4), min(24 / height, .4)
    limits = np.array((1 - margin_x, 1 - margin_y))
    edge = not front or np.any(np.abs(ndc) > limits)
    if edge:
        if np.linalg.norm(ndc) < 1e-8:
            ndc = np.array((0.0, -1.0))
        ndc = ndc / max(np.abs(ndc) / limits)
    return ((ndc[0] + 1) * width / 2, (ndc[1] + 1) * height / 2), edge


def project_position(position, camera, width, height):
    """Finite world point projection (w=1); rearward points get edge cues only."""
    point=np.asarray(position,dtype=float)
    if point.shape!=(3,) or not np.isfinite(point).all() or width<=0 or height<=0:
        return None
    direction=point-camera.position
    clip=camera.projection_matrix() @ camera.view_matrix() @ np.array((*point,1.0))
    if not np.isfinite(clip).all():
        return None
    if clip[3]<=camera.near or np.any(np.abs(clip[:2]/max(clip[3],1e-8))>1):
        return project_direction(direction,camera,width,height)
    ndc=clip[:2]/clip[3]
    return ((ndc[0]+1)*width/2,(ndc[1]+1)*height/2),False


def instrument_groups(vehicle, fcc=None, debug=False):
    """Read-only groups; H enables detailed force data rather than normal clutter."""
    atmospheric = vehicle.flight_state.environment is Environment.ATMOSPHERE
    aero = vehicle.aerodynamics
    speed = aero.airspeed if atmospheric else vehicle.speed
    heading = heading_degrees(vehicle)
    heading_text = '---' if heading is None else f'{int(round(heading)) % 360:03d}'
    left = [f'SPD {speed:.1f} M/S', f'ALT {vehicle.position[1]:.1f} M',
            f'VSI {vertical_speed(vehicle):+.1f} M/S', f'HDG {heading_text}',
            f'G {normal_g_load(vehicle):+.1f}',
            f'AOA {math.degrees(aero.alpha):+.1f} DEG' if atmospheric else 'AOA ---',
            f'BETA {math.degrees(aero.beta):+.1f} DEG' if atmospheric else 'BETA ---']
    right = [f'THR {vehicle.engine_throttle*100:.0f}%',
             f'VEC {vehicle.thrust_vector*100:.0f}%' if vehicle.flight_state.mode is VehicleMode.VTOL else 'VEC ---',
             f'MODE {vehicle.flight_state.mode.name}', f'ENV {vehicle.flight_state.environment.name}',
             'SAS ON' if fcc is None or fcc.stability_assist_enabled else 'SAS OFF',
             'HOV ON' if fcc is not None and fcc.hover_assist_enabled else 'HOV OFF']
    designation=getattr(vehicle,'unit_designation',None)
    if designation is not None:right.append(f'UNIT {designation}')
    transformation=getattr(vehicle, "transformation", None)
    if transformation is not None and transformation.active:
        right[2] = f"MODE {transformation.source.name} > {transformation.target.name}"
        right.append(f"XFORM {transformation.progress*100:.0f}%")
    battledroid=vehicle.flight_state.mode is VehicleMode.BATTLEDROID
    vtol=vehicle.flight_state.mode is VehicleMode.VTOL
    if vtol:
        ground=vehicle.ground_contact
        label=('CRASHED' if vehicle.flight_state.status is FlightStatus.CRASHED else
               'LANDED' if vehicle.is_grounded else 'LANDING' if atmospheric and ground.landing_approach else 'AIRBORNE')
        right.append(label)
    if battledroid:
        right.append('GROUND' if vehicle.is_grounded else 'AIRBORNE')
        if vehicle.is_grounded:left.append(f'WALK {np.linalg.norm(vehicle.velocity[[0,2]]):.1f} M/S')
    warnings = []
    if atmospheric and vehicle.flight_state.mode is VehicleMode.FIGHTER:
        alpha = abs(math.degrees(aero.alpha))
        if alpha >= PARAMETERS.stall_angle_deg:
            warnings.append('STALL')
        elif alpha >= PARAMETERS.stall_angle_deg * PARAMETERS.high_aoa_warning_fraction:
            warnings.append('HIGH AOA')
    if (battledroid or vtol) and vehicle.ground_contact.hard_landing_time>0:
        warnings.append('HARD LANDING')
    if vehicle.flight_state.status is not FlightStatus.FLYING and not vehicle.is_grounded:
        reset = 'F2 RESET' if vehicle.flight_state.mode in (VehicleMode.VTOL,VehicleMode.BATTLEDROID) else 'R RESET'
        warnings.append(vehicle.flight_state.status.name + ' - ' + reset)
    if fcc is not None and fcc.hover_assist_enabled and not fcc.hover_correction_active:
        warnings.append('HOV PILOT / LIMITED')
    detail = []
    if debug:
        detail = [f'CL {aero.lift_coefficient:.2f}', f'CD {aero.drag_coefficient:.2f}',
                  f'LIFT {np.linalg.norm(aero.lift_force)/1000:.1f} KN',
                  f'DRAG {np.linalg.norm(aero.drag_force)/1000:.1f} KN',
                  f'THRUST {np.linalg.norm(aero.thrust_force)/1000:.1f} KN',
                  f'PILOT THR {vehicle.throttle*100:.0f}%']
        if battledroid:
            g=vehicle.ground_contact
            detail=[f'GROUNDED {int(vehicle.is_grounded)} DEPTH {g.depth:.3f} M',
                    f'VSI {vehicle.velocity[1]:+.2f} HSPD {np.linalg.norm(vehicle.velocity[[0,2]]):.2f}',
                    f'NORMAL {g.normal[0]:.1f} {g.normal[1]:.1f} {g.normal[2]:.1f}',
                    f'FRICTION {np.linalg.norm(g.friction_force)/1000:.1f} KN',
                    f'THRUSTER {np.linalg.norm(aero.thrust_force+aero.maneuver_force)/1000:.1f} KN',
                    f'WANT {g.desired_velocity[0]:+.1f} {g.desired_velocity[2]:+.1f}',
                    f'ACTUAL {vehicle.velocity[0]:+.1f} {vehicle.velocity[2]:+.1f}']
        if vtol and atmospheric:
            g=vehicle.ground_contact
            detail.extend((f'FOOT L {g.left_foot_height:.3f} R {g.right_foot_height:.3f}',
                           f'CLR {g.clearance:.3f} DEPTH {g.depth:.3f}',
                           f'SUPPORT {np.linalg.norm(g.reaction_force)/1000:.1f} KN',
                           f'FRICTION {np.linalg.norm(g.friction_force)/1000:.1f} KN'))
        animator=getattr(vehicle,'locomotion',None)
        if battledroid and animator is not None:
            animation = getattr(vehicle, 'animation', None)
            if animation is not None:
                detail.append(f'STATE {animation.state} IMPACT {animation.severity:.2f}')
            detail.extend((f'LOC {animator.state}',
                           f'FWD {animator.forward_speed:+.1f} LAT {animator.lateral_speed:+.1f}',
                           f'GAIT {animator.gait_phase:.2f} CAD {animator.cadence:.2f}',
                           f'ANIM {animator.blend:.2f}'))
    return dict(left=left, right=right, warnings=warnings, debug=detail)


def instrument_labels(vehicle, fcc=None, debug=False):
    return [label for group in instrument_groups(vehicle, fcc, debug).values() for label in group]


class HUD:
    def __init__(self):
        self.width, self.height = 1280, 720
        self.mesh = None

    def initialize(self):
        self.mesh = Mesh([(0, 0, 0, *HUD_COLOR)] * 3)

    def resize(self, width, height):
        self.width, self.height = width, height

    def geometry(self, vehicle, camera, fcc=None, debug=False, combat=None):
        """Build overlay triangles without writing to simulation state."""
        vertices = []

        def triangle(a, b, c, color):
            vertices.extend(((*point, 0, *color) for point in (a, b, c)))

        def line(a, b, color=HUD_COLOR, thickness=1.5):
            a, b = np.array(a, dtype=float), np.array(b, dtype=float)
            delta = b - a
            normal = np.array((-delta[1], delta[0])) / max(np.linalg.norm(delta), 1e-8) * thickness / 2
            triangle(a + normal, a - normal, b - normal, color)
            triangle(a + normal, b - normal, b + normal, color)

        def cross(x, y, size=12):
            for a, b in (((x-size, y), (x-4, y)), ((x+4, y), (x+size, y)),
                         ((x, y-size), (x, y-4)), ((x, y+4), (x, y+size))):
                line(a, b)

        cross(self.width / 2, self.height / 2)
        marker = project_direction(vehicle.velocity, camera, self.width, self.height)
        if marker is not None:
            (x, y), edge = marker
            if edge:
                radial = np.array((x-self.width/2, y-self.height/2))
                radial /= np.linalg.norm(radial)
                side = np.array((-radial[1], radial[0]))
                tip = np.array((x, y))
                line(tip - radial*10 + side*5, tip, VELOCITY_COLOR)
                line(tip, tip - radial*10 - side*5, VELOCITY_COLOR)
            else:
                for index in range(24):
                    a, b = index * math.tau / 24, (index+1) * math.tau / 24
                    line((x+8*math.cos(a), y+8*math.sin(a)),
                         (x+8*math.cos(b), y+8*math.sin(b)), VELOCITY_COLOR)
                line((x-14, y), (x-8, y), VELOCITY_COLOR)
                line((x+8, y), (x+14, y), VELOCITY_COLOR)
        # Chase looks down at the craft: an extra small cue shows true nose bearing.
        if camera.mode == 'CHASE':
            heading = project_direction(vehicle.forward, camera, self.width, self.height)
            if heading is not None and not heading[1]:
                (x, y), _ = heading
                line((x-5, y-4), (x, y+3))
                line((x, y+3), (x+5, y-4))
        scale = min(3.0, self.width / 420, self.height / 260)

        def text(label, x, top, color=HUD_COLOR):
            for column, character in enumerate(label):
                for iy, bits in enumerate(FONT[character].split('/')):
                    for ix, bit in enumerate(bits):
                        if bit == '1':
                            px, py = x + (column*4+ix)*scale, top-(iy+1)*scale
                            triangle((px,py), (px+scale,py), (px+scale,py+scale), color)
                            triangle((px,py), (px+scale,py+scale), (px,py+scale), color)

        groups = instrument_groups(vehicle, fcc, debug)
        for row, label in enumerate(groups['left']):
            text(label, 12, self.height-12-row*8*scale)
        for row, label in enumerate(groups['right']):
            text(label, self.width-12-len(label)*4*scale, self.height-12-row*8*scale)
        for row, label in enumerate(groups['warnings']):
            text(label, (self.width-len(label)*4*scale)/2, self.height*.78-row*8*scale, VELOCITY_COLOR)
        for row, label in enumerate(groups['debug']):
            debug_x=self.width*.25 if vehicle.flight_state.mode is VehicleMode.BATTLEDROID else 12
            text(label, debug_x, (len(groups['debug'])-row)*8*scale+12)

        if combat is not None:
            radar=combat.radar
            track=radar.track
            target=radar.current_target
            target_color=(.2,.85,1.0)
            lead_color=(1.0,.35,1.0)
            manager=radar.target_manager
            missile_control=combat.missile_fire_control
            locked=missile_control.lock_state is LockState.LOCKED
            if locked:target_color=(1.0,.7,.15)

            def edge_cue(point,color):
                tip=np.array(point)
                radial=tip-np.array((self.width/2,self.height/2))
                length=np.linalg.norm(radial)
                if length<1e-8:return
                radial/=length
                side=np.array((-radial[1],radial[0]))
                line(tip-radial*12+side*6,tip,color,2)
                line(tip,tip-radial*12-side*6,color,2)

            if target is not None and target.alive:
                projected=project_position(target.position,camera,self.width,self.height)
                if projected is not None:
                    (x,y),edge=projected
                    if edge:
                        edge_cue((x,y),target_color)
                    else:
                        size=16
                        if locked:
                            points=((x,y+size),(x+size,y),(x,y-size),(x-size,y))
                            for index in range(4):line(points[index],points[(index+1)%4],target_color,2)
                        for a,b in (((x-size,y-size),(x+size,y-size)),
                                    ((x+size,y-size),(x+size,y+size)),
                                    ((x+size,y+size),(x-size,y+size)),
                                    ((x-size,y+size),(x-size,y-size))):line(a,b,target_color)
            if radar.solution is not None:
                lead=project_direction(radar.solution.required_direction,camera,self.width,self.height)
                if lead is not None:
                    (x,y),edge=lead
                    if edge:
                        edge_cue((x,y),lead_color)
                    else:
                        diamond=((x,y+10),(x+10,y),(x,y-10),(x-10,y))
                        for index in range(4):line(diamond[index],diamond[(index+1)%4],lead_color,2)
            target_index='--' if manager.index is None else f'{manager.index:02d}'
            combat_labels=[f'TGT {target_index}/{manager.count:02d}']
            if track is not None:
                combat_labels.extend((f'ID {track.target_id:02d}',f'RNG {track.range:.0f} M',f'CLS {track.closure:+.1f} M/S'))
            if track is not None and not track.in_range:combat_labels.append('RADAR RANGE')
            elif track is not None and radar.solution is None:combat_labels.append('NO SOLUTION')
            if debug and radar.solution is not None:combat_labels.append(f'TOF {radar.solution.intercept_time:.2f} S')
            for row,label in enumerate(combat_labels):
                text(label,12,self.height-12-(9+row)*8*scale,target_color)
            if radar.shoot:text('SHOOT',(self.width-5*4*scale)/2,self.height/2+35,lead_color)
            if combat.hit_flash>0:text('HIT',(self.width-3*4*scale)/2,self.height/2-25,lead_color)
            elif combat.miss_flash>0:text('MISS',(self.width-4*4*scale)/2,self.height/2-25,lead_color)
            status = getattr(combat, 'status', None)
            if status is not None:
                text(f'HOSTILES {status.hostile_count:02d} KILLS {combat.kills:02d}',
                     12, self.height-12-16*8*scale, target_color)
                if status.player_hit_flash > 0:
                    text(f'WARNING {status.player_hit_weapon} HIT {status.player_hits}',
                         self.width*.35, self.height*.62, (1,.2,.2))
                avionics = getattr(vehicle,'avionics',None)
                if avionics is not None and avionics.incoming:
                    text(f'MISSILE WARNING {len(avionics.incoming)}', self.width*.35, self.height*.67, (1,.2,.2))
                    estimates = [t.time_to_impact for t in avionics.incoming if t.time_to_impact is not None]
                    if estimates: text(f'TTI {min(estimates):.1f} S', self.width*.43, self.height*.64, (1,.4,.2))
            if debug and target is not None and getattr(target, 'hostile', False):
                pilot = target.pilot
                cmd = pilot.commands
                labels = [f'AI {pilot.state.name}', f'RNG {pilot.range_to_player:.0f} SPD {target.speed:.1f}',
                          f'AOA {math.degrees(target.aerodynamics.alpha):+.1f} THR {target.throttle*100:.0f}%',
                          f'CMD P {cmd.pitch:+.2f} R {cmd.roll:+.2f} Y {cmd.yaw:+.2f}',
                          'THREAT MISSILE' if pilot.missile_threat else 'THREAT NONE',
                          'TERRAIN RECOVER' if pilot.terrain_danger else 'TERRAIN SAFE',
                          f'DIR {pilot.desired_direction[0]:+.2f} {pilot.desired_direction[1]:+.2f} {pilot.desired_direction[2]:+.2f}']
                fc=target.combat.missile_fire_control
                labels.extend((f'{fc.selected_type.name} {fc.lock_state.name} {fc.lock_progress:.2f} S',
                               f'FLR {target.defenses.flares} CHF {target.defenses.chaff} ECM {int(target.ecm_enabled)}'))
                for row, label in enumerate(labels):
                    text(label, self.width*.52, self.height*.87-row*8*scale, (1,.4,.5))
            prefix='IR' if missile_control.selected_type is SeekerType.IR else 'RDR'
            lock_text={LockState.NO_TARGET:'NO TARGET',LockState.ACQUIRING:prefix+' ACQ',
                       LockState.LOCKED:prefix+' LOCK',LockState.OUT_OF_RANGE:'NO RNG'}[missile_control.lock_state]
            missile_labels=[f'WPN {missile_control.selected_type.name}',
                            f'RDR {missile_control.inventories[SeekerType.RADAR]:02d} IR {missile_control.inventories[SeekerType.IR]:02d}',lock_text,
                            'IN RNG' if missile_control.in_envelope else 'NO RNG']
            if combat.missiles:
                latest=combat.missiles[-1]
                missile_labels.extend(('MSL ACTIVE',f'TOF {latest.age:.1f} S'))
            if debug:missile_labels.append(f'ACQ {missile_control.lock_progress:.1f} S')
            for row,label in enumerate(missile_labels):
                text(label,self.width-12-len(label)*4*scale,self.height-12-(8+row)*8*scale,target_color)

        defenses = getattr(vehicle,'defenses',None)
        if defenses is not None:
            text(f'FLR {defenses.flares:02d} CHF {defenses.chaff:02d}',self.width-170,20)
            text('ECM ON' if defenses.ecm_enabled else 'ECM OFF',self.width-170,36)
        avionics = getattr(vehicle,'avionics',None)
        if avionics is not None:
            origin=np.array((self.width*.12,self.height*.25))
            r=min(45.,self.width*.06,self.height*.08)
            for i in range(32):
                a,b=i*math.tau/32,(i+1)*math.tau/32
                line(origin+r*np.array((math.sin(a),math.cos(a))),origin+r*np.array((math.sin(b),math.cos(b))))
            line(origin+(-4,0),origin+(4,0));line(origin+(0,-4),origin+(0,4))
            text('RWR',origin[0]-12,origin[1]+r+8)
            for threat in avionics.rwr:
                point=origin+r*.8*np.array((math.sin(threat.bearing),math.cos(threat.bearing)))
                color=(1,.15,.15) if threat.state is ThreatState.MISSILE else ((1,.7,.1) if threat.state is ThreatState.LOCK else (.2,.85,1))
                size=6 if threat.state is ThreatState.MISSILE else (5 if threat.state is ThreatState.LOCK else 3)
                line(point+(-size,0),point+(size,0),color,2)
                line(point+(0,-size),point+(0,size),color,2)
                if threat.state is not ThreatState.TRACK:
                    line(point+(-size,-size),point+(size,size),color,2)
            if debug:
                missiles=[t.source for t in avionics.incoming]
                if not missiles and combat is not None: missiles=combat.missiles
                if missiles:
                    missile=missiles[-1];tracked=missile.current_seeker_target
                    kind=getattr(tracked,'kind',None)
                    label='LOST' if missile.seeker_state.name in ('LOST','SEARCHING') else (kind.name if kind is not None else 'AIRCRAFT')
                    score=getattr(missile.seeker,'candidate_scores',{}).get(tracked,0.)
                    for i,label_text in enumerate((f'TYPE {missile.seeker_type.name} TRACK {label}',
                            f'SCORE {score:.2f} ORIG {missile.original_target.target_id}')):
                        text(label_text,self.width*.4,70+i*16,(1,.6,.2))

        # Dedicated attitude reference uses VEHICLE basis, never the observing camera.
        # It reports world-up orientation without commanding attitude corrections.
        center = np.array((self.width/2, self.height*.23))
        radius = min(60.0, self.width*.12, self.height*.14)
        pitch, bank = attitude_angles(vehicle)
        for index in range(40):
            a, b = index*math.tau/40, (index+1)*math.tau/40
            line(center+radius*np.array((math.cos(a),math.sin(a))),
                 center+radius*np.array((math.cos(b),math.sin(b))))
        line(center+(-12,0), center+(12,0), VELOCITY_COLOR)
        line(center+(0,-4), center+(0,4), VELOCITY_COLOR)
        if bank is not None:
            horizon_axis = np.array((math.cos(bank), math.sin(bank)))
            world_up = np.array((-math.sin(bank), math.cos(bank)))
            for reference in (-30,-20,-10,0,10,20,30):
                offset = (reference-math.degrees(pitch))*radius/45
                if abs(offset) < radius*.85:
                    width = radius*.70 if reference == 0 else radius*.35
                    middle = center+world_up*offset
                    line(middle-horizon_axis*width,middle+horizon_axis*width)
                    if reference == 0:
                        # Directional ground-side ticks distinguish an inverted horizon.
                        for side in (-1,1):
                            endpoint=middle+horizon_axis*width*side
                            line(endpoint,endpoint-world_up*radius*.13,VELOCITY_COLOR)
            pointer=center+world_up*radius
            line(pointer,pointer-world_up*8+horizon_axis*4)
            line(pointer,pointer-world_up*8-horizon_axis*4)
        for angle in (-60,-30,0,30,60):
            radians=math.radians(angle)
            radial=np.array((math.sin(radians),math.cos(radians)))
            line(center+radial*radius,center+radial*(radius+5))
        bank_text='---' if bank is None else f'{math.degrees(bank):+.0f}'
        attitude_label=f'P {math.degrees(pitch):+.0f} B {bank_text}'
        text(attitude_label,(self.width-len(attitude_label)*4*scale)/2,center[1]-radius-8)

        if vehicle.flight_state.mode is VehicleMode.VTOL:
            # This indicates THRUST FORCE: upward force means downward nozzle/exhaust.
            # It is a local forward/up diagram, not an absolute world-space direction.
            origin=np.array((self.width*.85,self.height*.23))
            length=min(45,self.width*.08,self.height*.1)
            line(origin,origin+(length,0));line(origin,origin+(0,length))
            vector=vehicle.thrust_vector
            direction=np.array((1-vector,vector),dtype=float);direction/=np.linalg.norm(direction)
            tip=origin+direction*length
            line(origin,tip,VELOCITY_COLOR,2.5)
            side=np.array((-direction[1],direction[0]))
            line(tip,tip-direction*7+side*4,VELOCITY_COLOR)
            line(tip,tip-direction*7-side*4,VELOCITY_COLOR)
            text('FWD',origin[0]+length+3,origin[1]+3)
            text('UP',origin[0]-4*scale,origin[1]+length+8*scale)
        return np.ascontiguousarray(vertices, dtype=np.float32)

    def render(self, vehicle, camera, shader, fcc=None, debug=False, combat=None):
        if self.width <= 0 or self.height <= 0:
            return
        vertices = self.geometry(vehicle, camera, fcc, debug, combat)
        projection = np.eye(4, dtype=np.float32)
        projection[0, 0], projection[1, 1] = 2 / self.width, 2 / self.height
        projection[0, 3] = projection[1, 3] = -1
        depth_enabled = GL.glIsEnabled(GL.GL_DEPTH_TEST)
        try:
            GL.glDisable(GL.GL_DEPTH_TEST)
            shader.use()
            shader.set_matrix('model', np.eye(4, dtype=np.float32))
            shader.set_matrix('view', np.eye(4, dtype=np.float32))
            shader.set_matrix('projection', projection)
            GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self.mesh.vbo)
            GL.glBufferData(GL.GL_ARRAY_BUFFER, vertices.nbytes, vertices, GL.GL_STREAM_DRAW)
            GL.glBindBuffer(GL.GL_ARRAY_BUFFER, 0)
            self.mesh.count = len(vertices)
            self.mesh.draw()
        finally:
            if depth_enabled:
                GL.glEnable(GL.GL_DEPTH_TEST)

    def close(self):
        if self.mesh is not None:
            self.mesh.close()
            self.mesh = None
