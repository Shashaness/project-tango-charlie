"""M19A clips, state transitions, telemetry and composition regressions."""
import unittest
from types import SimpleNamespace
import numpy as np
from engine.game import Game
from engine.quaternion import normalize
from game.animation_clips import AnimationClip, ClipPlayer, JointOffset, blend_pose, additive_pose
from game.battledroid_animation import STATES
from game.flight_state import FlightStatus as S, Environment as E, VehicleMode as M
from game.ground_contact import resolve_contact
from game.tc167_poses import rotation

class ClipTests(unittest.TestCase):
    def clip(self, loop=False):
        return AnimationClip({'name':'test','duration':1,'loop':loop,'targets':{'Joint':{
            'translation':[{'time':0,'value':[0,0,0]},{'time':1,'value':[2,0,0]}],
            'rotation':[{'time':0,'value':[0,0,0,2]},{'time':1,'value':rotation(90)}]}}})

    def test_interpolation_normalization_and_clamping(self):
        p=self.clip().sample(.5)['Joint']
        np.testing.assert_allclose(p.translation,(1,0,0))
        np.testing.assert_allclose(p.rotation,rotation(45),atol=1e-12)
        self.assertAlmostEqual(np.linalg.norm(p.rotation),1)
        self.assertEqual(self.clip().sample(4)['Joint'].translation,(2,0,0))

    def test_playback_loop_and_partition(self):
        player=ClipPlayer(self.clip());player.update(.4);player.update(.8)
        self.assertTrue(player.finished)
        loop=ClipPlayer(self.clip(True));p=loop.update(2.25)
        self.assertFalse(loop.finished);self.assertAlmostEqual(p['Joint'].translation[0],.5)
        for _ in range(100):loop.update(.01)
        np.testing.assert_allclose(loop.clip.sample(loop.time)['Joint'].translation,p['Joint'].translation,atol=1e-12)

    def test_blend_additive_and_no_mutation(self):
        base={'Joint':JointOffset((1,0,0),rotation(90))}
        layer={'Joint':JointOffset((0,1,0),rotation(30))}
        result=additive_pose(base,layer)
        np.testing.assert_allclose(result['Joint'].matrix(),base['Joint'].matrix()@layer['Joint'].matrix(),atol=1e-12)
        self.assertEqual(base['Joint'].translation,(1,0,0))
        self.assertAlmostEqual(np.linalg.norm(blend_pose(base,layer,.3)['Joint'].rotation),1)

    def test_invalid_keys(self):
        for value in (float('nan'),-1):
            with self.assertRaises(ValueError):self.clip().sample(value)
        with self.assertRaises(ValueError):normalize((0,0,0,0))

class AnimationTests(unittest.TestCase):
    def setUp(self):
        self.game=Game(enemy_count=0);self.game.setup_battledroid_test('ground')
        self.v=self.game.player_vehicle;self.a=self.v.animation
        self.p=SimpleNamespace(pitch=-1,roll=0)

    def sample(self,speed=0,dt=.05):
        self.v.velocity[:]=(0,0,-speed)
        self.a.clear_layer();self.v.locomotion.clear_layer()
        self.a.prepare(dt,self.v,self.p);self.v.locomotion.update(dt,self.v,self.p);self.a.update()
        return self.a.state

    def touchdown(self,impact):
        self.v.flight_state.status=S.FLYING;self.sample(dt=.01)
        self.v.position[1]=3.1;self.v.velocity[1]=-impact
        resolve_contact(self.v,3.2,supported=False,safe_speed=7,crash_speed=35,safe_tilt=45)
        self.sample(dt=0)

    def test_states_and_hysteresis(self):
        self.assertEqual(self.sample(),'IDLE')
        self.assertEqual(self.sample(2),'WALKING')
        self.assertEqual(self.sample(4.6),'RUNNING')
        # Falling speed selects stopping; steady speeds then demonstrate hysteresis.
        self.sample(4.3);self.assertEqual(self.sample(4.3),'RUNNING')
        self.sample(3.9);self.assertEqual(self.sample(3.9),'WALKING')
        self.p.pitch=0;self.assertEqual(self.sample(2),'STOPPING')
        self.sample(0,dt=.5);self.v.yaw_rate=.4
        self.assertEqual(self.sample(),'TURNING')
        self.assertEqual(set(STATES),{'AIRBORNE','LANDING','RECOVERY','IDLE','WALKING','RUNNING','TURNING','STOPPING'})

    def test_single_touchdown_and_pre_resolution_impact(self):
        self.touchdown(10)
        self.assertEqual(self.a.state,'LANDING');self.assertEqual(self.v.velocity[1],0)
        self.assertEqual(self.v.ground_contact.touchdown_impact_speed,10)
        for _ in range(30):
            resolve_contact(self.v,3.2,supported=True,safe_speed=7,crash_speed=35,safe_tilt=45)
            self.sample(dt=.01)
        self.assertEqual(self.a.landing_count,1);self.assertEqual(self.a.state,'RECOVERY')
        event=self.v.ground_contact.touchdown_id
        resolve_contact(self.v,3.2,supported=False,safe_speed=7,crash_speed=35,safe_tilt=45)
        self.assertEqual(self.v.ground_contact.touchdown_id,event)
        self.sample(dt=1);self.assertEqual(self.a.state,'IDLE')

    def test_severity_recovery_and_compression(self):
        self.touchdown(2);gentle=self.a.recovery_duration
        self.sample(dt=.08);low=self.a.pose['TC167Root'].translation[1]
        self.sample(dt=2);self.touchdown(20)
        self.assertGreater(self.a.recovery_duration,gentle)
        self.sample(dt=.08);self.assertLess(self.a.pose['TC167Root'].translation[1],low)
        self.assertGreater(self.v.ground_contact.hard_landing_time,0)

    def test_air_space_crash_and_no_phase_advance(self):
        self.sample(6,.2);phase=self.v.locomotion.gait_phase
        self.v.flight_state.status=S.FLYING
        self.assertEqual(self.sample(6),'AIRBORNE')
        self.assertEqual(self.v.locomotion.gait_phase,phase)
        self.v.flight_state.environment=E.SPACE;self.sample(6,.3)
        self.assertEqual(self.a.weight,0);self.assertEqual(self.v.locomotion.blend,0)
        self.v.flight_state.environment=E.ATMOSPHERE;self.touchdown(40)
        self.assertIs(self.v.flight_state.status,S.CRASHED);self.assertFalse(self.a._enabled)

    def test_no_accumulation_engine_attachment_and_physical_invariance(self):
        self.touchdown(10);self.sample(dt=.1)
        physical=[self.v.position.copy(),self.v.velocity.copy(),self.v.orientation.copy()]
        pose=[n.local_matrix.copy() for n in self.a.model.nodes]
        foot=self.a.model.find_node('LeftFoot');engine=self.a.model.find_node('LeftEngine')
        relative=np.linalg.inv(foot.world_matrix)@engine.world_matrix
        for _ in range(20):self.sample(dt=0)
        for n,b in zip(self.a.model.nodes,pose):np.testing.assert_allclose(n.local_matrix,b,atol=1e-12)
        np.testing.assert_allclose(np.linalg.inv(foot.world_matrix)@engine.world_matrix,relative,atol=1e-12)
        for current,b in zip((self.v.position,self.v.velocity,self.v.orientation),physical):np.testing.assert_array_equal(current,b)

    def test_landing_timing_frame_partition(self):
        self.touchdown(8);self.sample(dt=.5);time=self.a.time;pose=self.a.pose
        self.setUp();self.touchdown(8)
        for _ in range(60):self.sample(dt=.5/60)
        self.assertEqual(self.a.state,'RECOVERY');self.assertAlmostEqual(self.a.time,time)
        # Crossfade has finished; the same clip time yields exactly the same pose.
        for name in pose:np.testing.assert_allclose(self.a.pose[name].matrix(),pose[name].matrix(),atol=1e-12)

    def test_clip_and_fading_gait_respect_articulation_bounds(self):
        self.sample(6,.2);self.touchdown(20)
        for _ in range(30):
            self.sample(dt=.01)
            for name,limit in (('LeftLowerLeg',38),('LeftFoot',6),('LeftUpperArm',18)):
                node=self.a.model.find_node(name)
                delta=np.linalg.inv(node.configuration_matrix)@node.local_matrix
                angle=np.degrees(np.arccos(np.clip((np.trace(delta[:3,:3])-1)/2,-1,1)))
                self.assertLessEqual(angle,limit+1e-6)

    def test_transformation_restoration(self):
        self.touchdown(10);self.sample(dt=.1)
        self.v.transformation.request(M.VTOL)
        for _ in range(240):
            self.a.clear_layer();self.v.locomotion.clear_layer();self.v.transformation.update(1/120)
            self.a.prepare(1/120,self.v,self.p);self.v.locomotion.update(1/120,self.v,self.p);self.a.update()
        reference=Game(enemy_count=0);reference.player_vehicle.transformation.reset(M.VTOL)
        for n,r in zip(self.a.model.nodes,reference.player_vehicle.transformation.model.nodes):np.testing.assert_array_equal(n.local_matrix,r.local_matrix)
