"""Hidden-context model/world draw and cleanup checks, with optional PNG evidence.

Run from the project root: python -m tools.check_transformation_gl --output /tmp/m16
Requires the same display/window-server access as the game. No gameplay changes.
"""
import argparse
from pathlib import Path
import struct
import zlib
import tempfile
import time
import glfw
import numpy as np
from OpenGL import GL
from engine.game import Game
from engine.renderer import Renderer
from game.flight_state import VehicleMode


def capture(path):
    GL.glFinish()
    rgb=np.frombuffer(GL.glReadPixels(0,0,1280,720,GL.GL_RGB,GL.GL_UNSIGNED_BYTE),dtype=np.uint8).reshape(720,1280,3)[::-1]
    def chunk(kind,data):
        return struct.pack('>I',len(data))+kind+data+struct.pack('>I',zlib.crc32(kind+data)&0xffffffff)
    raw=b''.join(b'\0'+row.tobytes() for row in rgb)
    path.write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',1280,720,8,2,0,0,0))+chunk(b'IDAT',zlib.compress(raw))+chunk(b'IEND',b''))


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path)
    parser.add_argument('--locomotion',action='store_true')
    parser.add_argument('--animation',action='store_true')
    parser.add_argument('--world-environment',action='store_true')
    parser.add_argument('--terrain-test',action='store_true',help='render locally generated synthetic HGT terrain')
    parser.add_argument('--vtol-ground',action='store_true');args=parser.parse_args()
    if args.output:args.output.mkdir(parents=True,exist_ok=True)
    window=None;renderer=None;terrain_fixture=None;terrain_config=None
    if args.terrain_test:
        from game.terrain import TerrainConfig
        terrain_fixture=tempfile.TemporaryDirectory(prefix='tc-synthetic-hgt-')
        row,col=np.indices((1201,1201))
        samples=(1000+650*np.sin(col/1200*8*np.pi)*np.sin(row/1200*8*np.pi)).astype('>i2')
        samples.tofile(Path(terrain_fixture.name)/'N34W112.hgt')
        terrain_config=TerrainConfig(directory=Path(terrain_fixture.name))
    try:
        if not glfw.init():raise RuntimeError('GLFW initialization failed')
        for key,value in ((glfw.CONTEXT_VERSION_MAJOR,3),(glfw.CONTEXT_VERSION_MINOR,3),
                          (glfw.OPENGL_PROFILE,glfw.OPENGL_CORE_PROFILE),(glfw.OPENGL_FORWARD_COMPAT,glfw.TRUE),
                          (glfw.VISIBLE,glfw.FALSE)):
            glfw.window_hint(key,value)
        window=glfw.create_window(1280,720,'TC167 M16 validation',None,None)
        if not window:raise RuntimeError('OpenGL context creation failed')
        glfw.make_context_current(window)
        for iteration in range(3):
            game=Game(enemy_count=0,terrain_config=terrain_config);v=game.player_vehicle;c=v.transformation
            renderer=Renderer(game.world,v,game.camera,game.flight_controller.fcc,game.combat)
            renderer.initialize();renderer.resize(window,1280,720)
            handles=[(m.vao,m.vbo,m.ebo) for m in renderer.fighter_resources.meshes]
            def draw(label):
                renderer.render();GL.glFinish()
                if GL.glGetError()!=GL.GL_NO_ERROR:raise RuntimeError('OpenGL error during '+label)
                if args.output and iteration==0:capture(args.output/(label+'.png'))
                glfw.swap_buffers(window)
            if args.world_environment:
                if not GL.glIsEnabled(GL.GL_DEPTH_TEST) or not GL.glGetBooleanv(GL.GL_DEPTH_WRITEMASK):
                    raise RuntimeError('Environment requires depth testing and depth writes')
                if GL.glGetIntegerv(GL.GL_DEPTH_FUNC) != GL.GL_LESS:
                    raise RuntimeError('Unexpected scene depth comparison')
                if iteration == 0:
                    print('Environment depth bits:', int(GL.glGetFramebufferAttachmentParameteriv(
                        GL.GL_DRAW_FRAMEBUFFER, GL.GL_DEPTH, GL.GL_FRAMEBUFFER_ATTACHMENT_DEPTH_SIZE)))
                game.reset_runway();draw('WORLD_RUNWAY')
                v.position[:] = (0,450,950)
                game.camera.snap_to_vehicle();game.camera.follow(v,0)
                draw('WORLD_SKYLINE')
                v.position[:] = (58,10,-40)
                game.camera.snap_to_vehicle();game.camera.follow(v,0)
                draw('WORLD_STREET')
                # Moving views expose distance-dependent striping that a single
                # static near-ground screenshot can miss. Camera controls unchanged.
                for label, origin in (('STREET', (58,10,-40)), ('ALTITUDE', (0,900,1200)),
                                      ('AIRFIELD', (180,650,4100))):
                    for sample in range(6):
                        v.position[:] = np.asarray(origin)+(sample*.35,0.,-sample*2.)
                        game.camera.snap_to_vehicle();game.camera.follow(v,0)
                        draw('WORLD_MOVING_'+label+'_'+str(sample))
                # Inspection-only top view: show authoritative blocks, corridors,
                # park paths and district density without changing gameplay cameras.
                game.camera.mode='COCKPIT'
                game.camera.position[:]=(0,1600,-300)
                game.camera.forward[:]=(0,-1,0)
                game.camera.right[:]=(1,0,0)
                game.camera.up[:]=(0,0,-1)
                draw('WORLD_CITY_PLAN')
                game.reset_runway()
            if args.terrain_test:
                from game.flight_state import Environment
                v.flight_state.environment=Environment.ATMOSPHERE
                game.camera.mode='COCKPIT'
                renderer.show_axes=True
                for label,position,forward in (
                    ('TERRAIN_NEAR',(12000,1400,14000),(-.35,-.2,-1)),
                    ('TERRAIN_BOUNDARY',(10500,800,5000),(-1,-.12,-.3)),
                    ('TERRAIN_FAR',(0,14000,10000),(0,-1,-.4))):
                    game.camera.position[:]=position
                    direction=np.asarray(forward,float);direction/=np.linalg.norm(direction)
                    right=np.cross(direction,[0,1,0]);right/=np.linalg.norm(right)
                    game.camera.forward[:]=direction;game.camera.right[:]=right
                    game.camera.up[:]=np.cross(right,direction)
                    deadline=time.monotonic()+5
                    while time.monotonic()<deadline:
                        renderer.render()
                        if not renderer.terrain_resources.pending:break
                        time.sleep(.01)
                    draw(label)
                    stats=renderer.terrain_resources.stats
                    if stats['patches']<1 or stats['patches']>terrain_config.budget:
                        raise RuntimeError('Invalid terrain patch budget')
                    if iteration==0:print(label,stats)
                handles.extend((m.vao,m.vbo,m.ebo) for m in renderer.terrain_resources.cache.values())
                renderer.show_axes=False
                game.reset_runway()
            for mode in VehicleMode:
                c.reset(mode);v.flight_state.mode=mode
                game.camera.snap_to_vehicle();game.camera.follow(v,0)
                draw(mode.name)
            c.reset();v.flight_state.mode=VehicleMode.FIGHTER
            game.camera.snap_to_vehicle();game.camera.follow(v,0)
            for target in (VehicleMode.VTOL,VehicleMode.BATTLEDROID,VehicleMode.FIGHTER):
                c.request(target)
                sample=0
                while c.active:
                    c.update(.15);game.camera.follow(v,.15)
                    draw('TO_'+target.name+'_'+str(sample));sample+=1
            # A reversal must also render without replacing resources.
            c.request(VehicleMode.VTOL);c.update(.675);c.reverse();c.update(.2);draw('REVERSAL')
            c.reset(VehicleMode.BATTLEDROID);v.flight_state.mode=VehicleMode.BATTLEDROID
            game.camera.toggle_external_mode()
            game.camera.reset_orbit();game.camera.follow(v,0)
            for label,dx,dy in (('ORBIT_SIDE',360,0),('ORBIT_FRONT',360,0),
                                 ('ORBIT_TOP',0,10000),('ORBIT_UNDERSIDE',0,-20000)):
                game.camera.orbit_drag(dx,dy);game.camera.snap_to_vehicle()
                game.camera.follow(v,0);draw(label)
            for label,dx,dy in (('ORBIT_FRONT_QUARTER',-540,7),
                                 ('ORBIT_RIGHT',-360,0),('ORBIT_ABOVE_QUARTER',-540,146)):
                game.camera.reset_orbit();game.camera.follow(v,0)
                game.camera.orbit_drag(dx,dy);game.camera.snap_to_vehicle()
                game.camera.follow(v,0);draw(label)
            game.camera.orbit_zoom(-4);game.camera.follow(v,1);draw('ORBIT_ZOOM')
            if args.locomotion:
                game.setup_battledroid_test('ground')
                game.camera.reset_orbit();game.camera.follow(v,0)
                game.camera.orbit_drag(360,0);game.camera.orbit_zoom(-4)
                game.camera.follow(v,1)
                for label,pitch,roll,yaw in (('WALK',-1,0,0),('BACKWARD',1,0,0),
                                             ('STRAFE',0,1,0),('DIAGONAL',-1,1,0),
                                             ('SKID',0,0,0),('TURN',0,0,1)):
                    game.input.pitch=pitch;game.input.roll=roll;game.input.yaw=yaw
                    for sample in range(120):
                        game.update(1/30)
                        if sample in (30,60,90,119):draw('GAIT_'+label+'_'+str(sample))
                    game.camera.reset_orbit();game.camera.follow(v,0)
                    game.camera.orbit_drag(720,0);game.camera.orbit_zoom(-4)
                    game.camera.follow(v,1);draw('GAIT_'+label+'_FRONT')
                    game.camera.reset_orbit();game.camera.follow(v,0)
                    game.camera.orbit_drag(360,0);game.camera.orbit_zoom(-4)
                    game.camera.follow(v,1)
                game.input.pitch=-1;game.input.yaw=0
                for _ in range(90):game.update(1/30)
                game.input.jump_pressed=True;game.update(.1);draw('GAIT_TAKEOFF')
                game.input.jump_pressed=False;game.update(.3);draw('GAIT_AIRBORNE')
                c.cycle()
                for sample in range(12):
                    game.update(.15);draw('GAIT_TRANSFORM_'+str(sample))
            if args.animation:
                from engine.input import Input
                from game.flight_state import FlightStatus
                for label, impact in (('GENTLE', 2.), ('HARD', 16.)):
                    game.input = Input();game.setup_battledroid_test('ground')
                    v.position[1] = 3.5;v.velocity[1] = -impact
                    v.flight_state.status = FlightStatus.FLYING
                    game.camera.reset_orbit();game.camera.follow(v, 0)
                    game.camera.orbit_drag(360, 0);game.camera.orbit_zoom(-4)
                    game.camera.follow(v, 1)
                    seen = set()
                    for sample in range(240):
                        game.update(1/120)
                        seen.add(v.animation.state)
                        if sample in (0, 8, 16, 28, 48, 80, 120, 239):
                            draw('ANIMATION_'+label+'_'+str(sample))
                    if not {'AIRBORNE', 'LANDING', 'RECOVERY', 'IDLE'} <= seen:
                        raise RuntimeError('Missing landing animation states: '+str(seen))
            if args.vtol_ground:
                from game.flight_state import FlightStatus
                from engine.input import Input
                game.input=Input();renderer.show_axes=True
                for case in ('landing','ground','skim','hard','crash'):
                    game.setup_vtol_test(case)
                    game.camera.reset_orbit();game.camera.follow(v,0)
                    game.camera.orbit_drag(360,0);game.camera.orbit_zoom(-2)
                    game.camera.follow(v,1)
                    for sample in range(480 if case=='landing' else 120):
                        game.update(1/120)
                        if sample in (0,60,119,360,479):draw('VTOL_'+case.upper()+'_'+str(sample))
                    if case in ('landing','ground','skim','hard') and not v.is_grounded:
                        raise RuntimeError('VTOL preset did not land: '+case)
                    if case=='crash' and v.flight_state.status is not FlightStatus.CRASHED:
                        raise RuntimeError('VTOL catastrophic impact did not crash')
                game.setup_vtol_test('ground');game.input.strafe=1;game.input.yaw=.2
                game.update(1);draw('VTOL_THRUST_SKIM')
                game.input.strafe=game.input.yaw=0;v.throttle=1.
                game.update(.5);draw('VTOL_LIFTOFF')
                if v.is_grounded:raise RuntimeError('VTOL thrust failed to lift off')
                game.setup_vtol_test('ground');c.request(VehicleMode.BATTLEDROID)
                for sample in range(14):game.update(.15);draw('GROUND_G_TO_B_'+str(sample))
                if not v.is_grounded:raise RuntimeError('Ground Battledroid handoff failed')
                c.request(VehicleMode.VTOL)
                for sample in range(14):game.update(.15);draw('GROUND_B_TO_G_'+str(sample))
                if not v.is_grounded:raise RuntimeError('Ground VTOL handoff failed')
            renderer.close();renderer.close()
            for vao,vbo,ebo in handles:
                if GL.glIsVertexArray(vao) or GL.glIsBuffer(vbo) or (ebo and GL.glIsBuffer(ebo)):
                    raise RuntimeError('Model GPU handle survived close')
            if GL.glGetError()!=GL.GL_NO_ERROR:raise RuntimeError('Cleanup OpenGL error')
        print('PASS: 3 repeated canonical uploads, all endpoints/transitions/reversal, no GL errors, all VAO/VBO/EBO handles deleted')
    finally:
        if renderer is not None:renderer.close()
        if window is not None:glfw.destroy_window(window)
        glfw.terminate()
        if terrain_fixture is not None:terrain_fixture.cleanup()

if __name__=='__main__':main()
