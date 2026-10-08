"""Camera-only orbit state, GLFW gestures and unchanged flight regressions."""
import math
import unittest
from unittest.mock import patch
import glfw
import numpy as np
from engine.camera import Camera,ORBIT_PITCH_LIMIT,ORBIT_MIN_DISTANCE,ORBIT_MAX_DISTANCE
from engine.camera_input import CameraInput
from engine.game import Game
from game.player_vehicle import PlayerVehicle
from game.flight_state import VehicleMode

class OrbitTests(unittest.TestCase):
    def setUp(self):
        self.vehicle=PlayerVehicle();self.camera=Camera();self.camera.follow(self.vehicle,0);self.camera.toggle_external_mode()

    def test_default_chase_exact_and_reset_mode_defaults(self):
        v=self.vehicle;c=self.camera
        c.toggle_external_mode()
        self.assertEqual(c.label,'CHASE')
        np.testing.assert_allclose(c.position,v.position-v.forward*24+v.up*7)
        c.toggle_external_mode()
        c.orbit_drag(200,100);c.orbit_zoom(3);c.follow(v,.1)
        c.reset_orbit();c.follow(v,0)
        np.testing.assert_allclose(c.position,v.position-v.forward*24+v.up*7)
        self.assertEqual(c.orbit_yaw,0);self.assertAlmostEqual(c.orbit_distance,25)
        v.flight_state.mode=VehicleMode.BATTLEDROID;c.follow(v,1)
        c.orbit_drag(200,0);c.reset_orbit();c.follow(v,0)
        np.testing.assert_allclose(c.position,v.position-v.forward*12+v.up*4)
        self.assertAlmostEqual(c.orbit_distance,math.hypot(12,4))

    def test_drag_changes_only_camera_and_yaw_has_full_rotation(self):
        v=self.vehicle;c=self.camera;v.velocity[:]=(10,8,-50)
        p,vel,o=v.position.copy(),v.velocity.copy(),v.orientation.copy()
        c.orbit_drag(720,0);c.follow(v,1)
        self.assertAlmostEqual(c.orbit_yaw,-math.pi)
        self.assertLess(c.position[2],v.position[2])
        c.orbit_drag(720,0);c.follow(v,1)
        self.assertAlmostEqual(c.orbit_yaw,-2*math.pi)
        self.assertGreater(c.position[2],v.position[2])
        np.testing.assert_array_equal(v.position,p);np.testing.assert_array_equal(v.velocity,vel);np.testing.assert_array_equal(v.orientation,o)

    def test_pitch_clamps_both_poles_and_basis_is_safe(self):
        for dy,expected in ((10000,ORBIT_PITCH_LIMIT),(-20000,-ORBIT_PITCH_LIMIT)):
            self.camera.orbit_drag(100,dy);self.camera.follow(self.vehicle,1)
            self.assertEqual(self.camera.orbit_pitch,expected)
            np.testing.assert_allclose(self.camera.orientation.T@self.camera.orientation,np.eye(3),atol=1e-12)
            self.assertTrue(np.isfinite(self.camera.view_matrix()).all())

    def test_scroll_clamps_and_does_not_touch_throttle(self):
        v=self.vehicle;v.throttle=.7
        self.camera.orbit_zoom(10000);self.assertEqual(self.camera.orbit_distance,ORBIT_MIN_DISTANCE)
        self.camera.orbit_zoom(-10000);self.assertEqual(self.camera.orbit_distance,ORBIT_MAX_DISTANCE)
        self.assertEqual(v.throttle,.7)

    def test_manual_view_world_angle_persists_through_translation_rotation_modes(self):
        v=self.vehicle;c=self.camera
        c.orbit_drag(360,60);c.orbit_zoom(2);c.follow(v,10)
        offset=c.position-v.position;distance=c.orbit_distance
        v.position+=(1000,40,-700);v.rotate(yaw=1.5,roll=2)
        v.flight_state.mode=VehicleMode.BATTLEDROID;c.follow(v,1)
        np.testing.assert_allclose(c.position-v.position,offset,atol=1e-10)
        self.assertEqual(c.orbit_distance,distance)
        np.testing.assert_allclose(c.forward,(v.position-c.position)/np.linalg.norm(v.position-c.position),atol=1e-12)

    def test_cockpit_unchanged_mouse_inactive_and_previous_orbit_restored(self):
        v=self.vehicle;c=self.camera;c.orbit_drag(300,30);c.orbit_zoom(2);c.follow(v,10)
        state=(c.orbit_yaw,c.orbit_pitch,c.orbit_distance);offset=c.position-v.position
        c.toggle_mode();v.rotate(roll=.5);c.orbit_drag(200,300);c.orbit_zoom(10);c.follow(v,.1)
        self.assertEqual(state,(c.orbit_yaw,c.orbit_pitch,c.orbit_distance))
        np.testing.assert_array_equal(c.position,v.position);np.testing.assert_array_equal(c.orientation,v.orientation)
        c.toggle_mode();c.follow(v,.1)
        np.testing.assert_allclose(c.position-v.position,offset,atol=1e-12)

    def test_mouse_callbacks_trackpad_drag_scroll_home_and_release_no_jump(self):
        i=CameraInput();window=object();c=self.camera
        with patch('engine.camera_input.glfw.set_scroll_callback') as register:
            i.install(window);register.assert_called_once_with(window,i.on_scroll)
        with patch('engine.camera_input.glfw.get_cursor_pos',side_effect=[(10,20),(30,50),(700,800),(900,900)]),patch('engine.camera_input.glfw.get_mouse_button',side_effect=[glfw.PRESS,glfw.PRESS,glfw.RELEASE,glfw.PRESS]),patch('engine.camera_input.glfw.get_key',return_value=glfw.RELEASE):
            i.poll(window);self.assertEqual(i.drag,(0,0));i.apply(c)
            i.poll(window);self.assertEqual(i.drag,(20,30));i.apply(c)
            self.assertTrue(c._manual_orbit)
            i.poll(window);i.apply(c);yaw=c.orbit_yaw
            i.poll(window);i.apply(c);self.assertEqual(c.orbit_yaw,yaw)
        i.on_scroll(window,0,2);i.apply(c);self.assertEqual(i.scroll,0)
        with patch('engine.camera_input.glfw.get_cursor_pos',return_value=(0,0)),patch('engine.camera_input.glfw.get_mouse_button',return_value=glfw.RELEASE),patch('engine.camera_input.glfw.get_key',side_effect=lambda window,key:glfw.PRESS if key==glfw.KEY_HOME else glfw.RELEASE):
            i.poll(window);self.assertTrue(i.reset_pressed);i.apply(c);self.assertTrue(c._manual_orbit)
            i.poll(window);self.assertFalse(i.reset_pressed)

    def test_cockpit_consumes_scroll_without_deferred_zoom(self):
        i=CameraInput();c=self.camera;c.toggle_mode();distance=c.orbit_distance
        i.on_scroll(None,0,4);i.drag=(30,50);i.apply(c)
        c.toggle_mode();i.apply(c);self.assertEqual(c.orbit_distance,distance);self.assertTrue(c._manual_orbit)

    def test_transform_and_inventories_untouched_by_orbit(self):
        g=Game(enemy_count=0);v=g.player_vehicle;c=v.transformation;c.cycle();c.update(.5)
        state=(c.source,c.target,c.progress,c.configuration,v.throttle)
        matrices=[n.local_matrix.copy() for n in c.model.nodes]
        inventory=g.combat.missile_fire_control.inventories.copy();target=g.combat.radar.current_target
        g.camera.toggle_external_mode()
        g.camera_input.drag=(700,300);g.camera_input.scroll=3;g.update(0)
        self.assertEqual(state,(c.source,c.target,c.progress,c.configuration,v.throttle))
        for node,expected in zip(c.model.nodes,matrices):np.testing.assert_array_equal(node.local_matrix,expected)
        self.assertEqual(inventory,g.combat.missile_fire_control.inventories);self.assertIs(g.combat.radar.current_target,target)

    def test_smoothing_is_frame_rate_independent_and_radius_never_crosses_root(self):
        results=[]
        for fps in (30,60,144):
            c=Camera();c.follow(self.vehicle,0);c.toggle_external_mode();c.orbit_drag(720,100);c.orbit_zoom(2)
            for _ in range(fps):
                c.follow(self.vehicle,1/fps)
                self.assertGreaterEqual(np.linalg.norm(c.position-self.vehicle.position),ORBIT_MIN_DISTANCE-1e-10)
            results.append(c.position.copy())
        for result in results[1:]:np.testing.assert_allclose(result,results[0],atol=1e-10)

    def test_backspace_reset_alias_for_compact_keyboards(self):
        i=CameraInput();self.camera.orbit_drag(30,10)
        with patch('engine.camera_input.glfw.get_cursor_pos',return_value=(0,0)),patch('engine.camera_input.glfw.get_mouse_button',return_value=glfw.RELEASE),patch('engine.camera_input.glfw.get_key',side_effect=lambda window,key:glfw.PRESS if key==glfw.KEY_BACKSPACE else glfw.RELEASE):
            i.poll(None);self.assertTrue(i.reset_pressed);i.apply(self.camera)
        self.assertTrue(self.camera._manual_orbit)


class CameraModeTests(unittest.TestCase):
    def test_default_chase_mouse_inactive_and_continuous_vehicle_follow(self):
        v=PlayerVehicle();c=Camera();c.follow(v,0)
        self.assertEqual(c.label,'CHASE');self.assertFalse(c._manual_orbit)
        c.orbit_drag(720,100);c.orbit_zoom(4)
        self.assertEqual(c.external_camera_mode,'CHASE');self.assertFalse(c._manual_orbit)
        for _ in range(4):
            v.rotate(yaw=.6,pitch=.2,roll=.1);v.position+=v.forward*5
            c.follow(v,10)
            np.testing.assert_allclose(c.position,v.position-v.forward*24+v.up*7,atol=1e-10)

    def test_dolly_entry_captures_actual_smoothed_chase_offset(self):
        v=PlayerVehicle();c=Camera();c.follow(v,0)
        v.rotate(yaw=.5);v.position+=(3,2,-4);c.follow(v,.02)
        before=c.position.copy();c.toggle_external_mode();c.follow(v,0)
        np.testing.assert_allclose(c.position,before,atol=1e-12)
        self.assertEqual(c.label,'DOLLY');self.assertTrue(c._manual_orbit)
        offset=c.position-v.position
        v.rotate(yaw=1,roll=.7);v.position+=(50,20,-30);c.follow(v,1)
        np.testing.assert_allclose(c.position-v.position,offset,atol=1e-10)
        c.toggle_external_mode();c.follow(v,0)
        np.testing.assert_allclose(c.position,v.position-v.forward*24+v.up*7,atol=1e-12)
        v.rotate(yaw=.6);c.follow(v,10)
        np.testing.assert_allclose(c.position,v.position-v.forward*24+v.up*7,atol=1e-10)

    def test_zero_key_edges_hold_release_and_cockpit_external_restore(self):
        i=CameraInput();c=Camera();v=PlayerVehicle();c.follow(v,0)
        down=[False]
        with patch('engine.camera_input.glfw.get_cursor_pos',return_value=(0,0)),patch('engine.camera_input.glfw.get_mouse_button',return_value=glfw.RELEASE),patch('engine.camera_input.glfw.get_key',side_effect=lambda window,key:glfw.PRESS if key==glfw.KEY_0 and down[0] else glfw.RELEASE):
            down[0]=True;i.poll(None);i.apply(c);self.assertEqual(c.label,'DOLLY')
            i.poll(None);i.apply(c);self.assertEqual(c.label,'DOLLY')
            c.toggle_mode();self.assertEqual(c.label,'COCKPIT');c.follow(v,0)
            np.testing.assert_array_equal(c.orientation,v.orientation)
            c.toggle_mode();self.assertEqual(c.label,'DOLLY')
            down[0]=False;i.poll(None);i.apply(c)
            down[0]=True;i.poll(None);i.apply(c);self.assertEqual(c.label,'CHASE')
            c.toggle_mode();c.toggle_mode();self.assertEqual(c.label,'CHASE')

    def test_reset_dolly_stays_dolly_and_mouse_in_chase_is_consumed(self):
        i=CameraInput();c=Camera();v=PlayerVehicle();c.follow(v,0)
        i.drag=(100,100);i.scroll=2;i.apply(c)
        self.assertEqual(c.label,'CHASE');self.assertFalse(c._manual_orbit)
        c.toggle_external_mode();c.orbit_drag(200,60);c.orbit_zoom(2)
        c.reset_orbit();c.follow(v,0)
        self.assertEqual(c.label,'DOLLY');self.assertTrue(c._manual_orbit)
        np.testing.assert_allclose(c.position,v.position-v.forward*24+v.up*7)
