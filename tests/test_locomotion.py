"""M17 visual gait, canonical-pose preservation and physical invariance."""
import math
import unittest
from types import SimpleNamespace
import numpy as np
from engine.game import Game
from engine.hud import instrument_labels
from game.flight_state import VehicleMode as M,Environment as E,FlightStatus as S
from game.battledroid_locomotion import *

class LocomotionTests(unittest.TestCase):
    def setUp(self):
        self.g=Game(enemy_count=0);self.g.setup_battledroid_test('ground')
        self.v=self.g.player_vehicle;self.a=self.v.locomotion
        self.p=SimpleNamespace(pitch=-1,roll=0)
        self.base=[n.local_matrix.copy() for n in self.a.model.nodes]

    def sample(self,speed=2,dt=.2):
        self.v.velocity[:]=(0,0,-speed);self.a.update(dt,self.v,self.p)

    def test_idle_walk_run_labels_and_actual_velocity(self):
        self.sample(0);self.assertEqual(self.a.state,'IDLE')
        self.sample(2);self.assertEqual(self.a.state,'WALK')
        self.sample(6);self.assertEqual(self.a.state,'RUN')
        self.sample(.1);self.assertEqual(self.a.state,'IDLE')
        self.assertTrue(any(x.startswith('LOC ') for x in instrument_labels(self.v,debug=True)))
        self.assertFalse(any(x.startswith('LOC ') for x in instrument_labels(self.v)))

    def test_phase_is_frame_independent_and_wraps(self):
        self.v.velocity[:]=(0,0,-3)
        self.a.update(5,self.v,self.p);phase=self.a.gait_phase
        self.a.gait_phase=0
        for _ in range(600):self.a.update(5/600,self.v,self.p)
        self.assertAlmostEqual(self.a.gait_phase,phase,places=10)
        self.assertTrue(0<=phase<math.tau)

    def test_cadence_is_speed_dependent_and_bounded(self):
        self.sample(.4);slow=self.a.cadence
        self.sample(3);walk=self.a.cadence
        self.sample(1000);fast=self.a.cadence
        self.assertLess(slow,walk);self.assertLess(walk,fast)
        self.assertLessEqual(fast,RUN_MAX_CADENCE)

    def test_opposite_hips_arms_and_recovery_knees(self):
        self.sample(2)
        j=self.a.joint_angles
        self.assertAlmostEqual(j['LeftUpperLeg'][0],-j['RightUpperLeg'][0])
        self.assertLess(j['LeftUpperLeg'][0]*j['LeftUpperArm'][0],0)
        self.assertAlmostEqual(j['LeftUpperArm'][0],-j['RightUpperArm'][0])
        self.assertNotEqual(j['LeftLowerLeg'][0],j['RightLowerLeg'][0])
        for side in ('Left','Right'):
            self.assertTrue(0<=j[side+'LowerLeg'][0]<=RUN_KNEE_FLEX)
            self.assertLessEqual(abs(j[side+'Foot'][0]),ANKLE_AMPLITUDE)

    def test_forward_backward_sign_and_lateral_diagonal_response(self):
        self.sample(2);self.a.gait_phase=.7;self.a.update(0,self.v,self.p)
        f=self.a.joint_angles['LeftUpperLeg'][0]
        self.v.velocity[:]=(0,0,2);self.a.update(0,self.v,self.p)
        self.assertAlmostEqual(self.a.joint_angles['LeftUpperLeg'][0],-f)
        self.v.velocity[:]=(3,0,0);self.p.roll=1;self.a.update(0,self.v,self.p)
        self.assertAlmostEqual(self.a.joint_angles['LeftUpperLeg'][0],0)
        self.assertNotEqual(self.a.joint_angles['LeftUpperLeg'][1],0)
        self.v.velocity[:]=(4,0,-4);self.a.update(.1,self.v,self.p)
        x,y,_=self.a.joint_angles['LeftUpperLeg']
        self.assertLessEqual(math.hypot(x,y),RUN_HIP_SWING)
        self.v.rotate(yaw=math.pi/2);self.v.velocity[:]=self.v.forward*2
        self.a.update(0,self.v,self.p);self.assertAlmostEqual(self.a.forward_speed,2,places=6)
        self.assertAlmostEqual(self.a.lateral_speed,0,places=6)

    def test_strafe_feet_remain_separated_through_cycle(self):
        self.v.velocity[:]=(4,0,0);self.p.roll=1
        for phase in np.linspace(0,math.tau,25):
            self.a.gait_phase=phase;self.a.update(.0,self.v,self.p)
            # Ramp once, then sample fixed phase without advancing time.
            self.a.blend=1.;self.a.update(0,self.v,self.p)
            left=self.a.model.find_node('LeftFoot').world_matrix[0,3]
            right=self.a.model.find_node('RightFoot').world_matrix[0,3]
            self.assertGreater(right-left,1.7)
            edges=[]
            for side in ('Left','Right'):
                foot=self.a.model.find_node(side+'Foot');points=[]
                for node in foot.children:
                    for index in node.meshes:
                        p=self.a.model.primitives[index].positions
                        points.extend((node.world_matrix@np.column_stack((p,np.ones(len(p)))).T).T[:,0])
                edges.append((min(points),max(points)))
            self.assertGreater(edges[1][0]-edges[0][1],.05)

    def test_walk_run_threshold_has_continuous_offsets(self):
        self.sample(4.5-1e-6);self.a.gait_phase=.7;self.a.update(0,self.v,self.p)
        before=self.a.nodes['LeftUpperLeg'].local_matrix.copy()
        self.v.velocity[2]=-(4.5+1e-6);self.a.update(0,self.v,self.p)
        np.testing.assert_allclose(before,self.a.nodes['LeftUpperLeg'].local_matrix,atol=1e-6)

    def test_exact_idle_no_accumulation_and_start_stop_blend(self):
        self.sample(2,.05);self.assertAlmostEqual(self.a.blend,.25)
        self.a.update(.15,self.v,self.p);self.assertEqual(self.a.blend,1)
        pose=[n.local_matrix.copy() for n in self.a.model.nodes]
        for _ in range(10):self.a.update(0,self.v,self.p)
        for n,p in zip(self.a.model.nodes,pose):np.testing.assert_allclose(n.local_matrix,p,atol=1e-12)
        self.sample(0,.05);self.assertAlmostEqual(self.a.blend,.75)
        self.sample(0,.2);self.assertEqual(self.a.offsets,{})
        for n,p in zip(self.a.model.nodes,self.base):np.testing.assert_array_equal(n.local_matrix,p)

    def test_passive_skid_fades_without_phase_cycling(self):
        self.sample(6);phase=self.a.gait_phase
        self.p.pitch=0;self.a.update(.05,self.v,self.p)
        self.assertEqual(self.a.state,'SKID');self.assertEqual(self.a.gait_phase,phase)
        self.assertLess(self.a.blend,1)
        self.a.update(.3,self.v,self.p);self.assertEqual(self.a.blend,0)

    def test_airborne_and_space_no_gait_and_landing_ramp(self):
        self.sample(3);phase=self.a.gait_phase
        self.v.flight_state.status=S.FLYING;self.a.update(.05,self.v,self.p)
        self.assertEqual(self.a.gait_phase,phase);self.assertAlmostEqual(self.a.blend,.75)
        self.a.update(.2,self.v,self.p);self.assertEqual(self.a.blend,0)
        self.v.flight_state.status=S.GROUNDED;self.v.flight_state.environment=E.SPACE
        self.sample(100);self.assertEqual(self.a.blend,0)
        self.v.flight_state.environment=E.ATMOSPHERE;self.sample(2,.02)
        self.assertAlmostEqual(self.a.blend,.1)

    def test_turn_in_place_uses_actual_yaw_rate(self):
        self.v.yaw_rate=.4;self.sample(0)
        self.assertEqual(self.a.state,'TURN');self.assertGreater(self.a.cadence,0)
        self.assertNotEqual(self.a.joint_angles['LeftUpperLeg'][0],0)

    def test_animation_is_read_only_for_all_physical_state(self):
        self.v.velocity[:]=(2,0,-3);self.v.throttle=.4
        arrays=[self.v.position,self.v.velocity,self.v.orientation,self.v.ground_contact.normal,
                self.v.ground_contact.friction_force,self.v.ground_contact.desired_velocity]
        before=[v.copy() for v in arrays];status=self.v.flight_state.status
        for _ in range(30):self.a.update(.01,self.v,self.p)
        for v,b in zip(arrays,before):np.testing.assert_array_equal(v,b)
        self.assertEqual(self.v.throttle,.4);self.assertIs(self.v.flight_state.status,status)

    def test_transform_fades_and_leaves_exact_vtol_and_endpoint(self):
        self.sample(2);self.v.transformation.request(M.VTOL)
        self.a.clear_layer();self.v.transformation.update(.05);self.a.update(.05,self.v,self.p)
        self.assertAlmostEqual(self.a.blend,.75)
        self.a.clear_layer();self.v.transformation.update(2);self.a.update(2,self.v,self.p)
        ref=Game(enemy_count=0);ref.player_vehicle.transformation.reset(M.VTOL)
        for n,r in zip(self.a.model.nodes,ref.player_vehicle.transformation.model.nodes):np.testing.assert_array_equal(n.local_matrix,r.local_matrix)
        self.v.transformation.request(M.BATTLEDROID)
        self.a.update(0,self.v,self.p);self.a.clear_layer();self.v.transformation.update(2)
        self.v.flight_state.status=S.GROUNDED;self.a.update(.01,self.v,self.p)
        self.assertEqual(self.a.blend,0)
        for n,r in zip(self.a.model.nodes,self.base):np.testing.assert_array_equal(n.local_matrix,r)

    def test_fighter_vtol_and_disabled_are_canonical(self):
        for mode in (M.FIGHTER,M.VTOL,M.BATTLEDROID):
            self.a.clear_layer();self.v.transformation.reset(mode);self.v.flight_state.mode=mode
            base=[n.local_matrix.copy() for n in self.a.model.nodes]
            self.a.enabled=False;self.sample(6)
            for n,b in zip(self.a.model.nodes,base):np.testing.assert_array_equal(n.local_matrix,b)

    def test_engine_collar_attachment_follows_animated_foot_once(self):
        foot=self.a.model.find_node('LeftFoot');engine=self.a.model.find_node('LeftEngine')
        relative=np.linalg.inv(foot.world_matrix)@engine.world_matrix
        self.sample(3)
        np.testing.assert_allclose(np.linalg.inv(foot.world_matrix)@engine.world_matrix,relative,atol=1e-11)
        marker=self.a.model.find_node('LeftEngineExhaust')
        np.testing.assert_allclose(marker.world_matrix,engine.world_matrix@marker.local_matrix,atol=1e-12)

    def test_enabled_disabled_full_game_physical_trajectory_matches(self):
        other=Game(enemy_count=0,locomotion_animation=False);other.setup_battledroid_test('ground')
        for i in range(160):
            for game in (self.g,other):
                game.input.pitch=-1 if i<90 else 0
                game.input.roll=.5 if 30<i<70 else 0
                game.input.yaw=.5 if 70<i<100 else 0
                game.input.jump_pressed=100<=i<104
                game.update(1/30)
            for field in ('position','velocity','orientation'):
                np.testing.assert_array_equal(getattr(self.v,field),getattr(other.player_vehicle,field))
            self.assertEqual(self.v.throttle,other.player_vehicle.throttle)
            self.assertIs(self.v.flight_state.status,other.player_vehicle.flight_state.status)
