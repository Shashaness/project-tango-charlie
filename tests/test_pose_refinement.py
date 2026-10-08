"""M16.2 lower hip carriers, compact backpack and fresh nozzle descendants."""
import unittest
import numpy as np
from game.transformation import TransformationController
from game.tc167_poses import (FIGHTER_POSE,VTOL_POSE,BATTLEDROID_POSE,rotation,DURATIONS)
from game.flight_state import VehicleMode as M
from game.player_vehicle import PlayerVehicle
from game.transformation_test import TransformationTest

class PoseRefinementTests(unittest.TestCase):
    def test_approved_fighter_and_vtol_pose_tables_are_preserved(self):
        self.assertEqual(FIGHTER_POSE,{})
        expected={}
        for side,label in ((1,'Left'),(-1,'Right')):
            expected.update({label+'Intake':((side*1.6,-.1,2),rotation(x=-8)),
                             label+'UpperLeg':((0,0,-1.5),rotation(x=-57)),
                             label+'LowerLeg':((0,0,-2.6),rotation(x=-35)),
                             label+'Foot':((0,0,-2.6),rotation(x=-80)),
                             label+'Wing':((side*1.3,0,.5),rotation(z=side*8)),
                             label+'Tail':((side*.8,.3,-6),rotation(x=8))})
        self.assertEqual(set(expected),set(VTOL_POSE))
        for name,(t,q) in expected.items():
            self.assertEqual(VTOL_POSE[name].translation,t)
            np.testing.assert_allclose(VTOL_POSE[name].rotation,q,atol=1e-14)
            self.assertEqual(VTOL_POSE[name].scale,(1,1,1))

    def test_intake_parent_is_lower_than_shoulders_and_descendants_inherit(self):
        c=TransformationController();c.reset(M.BATTLEDROID)
        for side in ('Left','Right'):
            intake=c.model.find_node(side+'Intake');shoulder=c.model.find_node(side+'Shoulder')
            self.assertLess(intake.world_matrix[1,3],shoulder.world_matrix[1,3]-1)
            self.assertLess(BATTLEDROID_POSE[side+'Intake'].translation[1],VTOL_POSE[side+'Intake'].translation[1])
            self.assertLess(BATTLEDROID_POSE[side+'Intake'].translation[1],2.62-1.5)
            self.assertLess(abs(shoulder.world_matrix[0,3]),2.25)
            self.assertLess(abs(intake.world_matrix[0,3]),1.3)
            node=c.model.find_node(side+'Foot')
            self.assertIs(node.parent.parent.parent,intake)
            for name in ('UpperLeg','LowerLeg','Foot'):
                n=c.model.find_node(side+name)
                np.testing.assert_allclose(n.world_matrix,n.parent.world_matrix@n.local_matrix,atol=1e-12)
            # Keep the unchanged physical contact datum without moving the vehicle.
            p=c.model.primitives[node.children[0].meshes[0]].positions
            points=(node.children[0].world_matrix@np.column_stack((p,np.ones(len(p)))).T).T
            self.assertAlmostEqual(points[:,1].min(),-3.2,delta=.01)

    def test_wings_fold_as_back_planes_without_scaling_or_hiding(self):
        c=TransformationController();c.reset(M.BATTLEDROID)
        for side in ('Left','Right'):
            node=c.model.find_node(side+'Wing');pose=BATTLEDROID_POSE[node.name]
            self.assertEqual(pose.scale,(1,1,1))
            surface=next(child for child in node.children if child.meshes)
            p=c.model.primitives[surface.meshes[0]].positions
            pts=(surface.world_matrix@np.column_stack((p,np.ones(len(p)))).T).T[:,:3]
            self.assertLess(np.ptp(pts[:,2]),.25)  # folded wing is flat against back
            self.assertLess(np.max(np.abs(pts[:,0])),2.5)
            self.assertGreater(pts[:,2].min(),1.)
            self.assertGreater(len(p),0)

    def test_exact_reverse_vtol_endpoint_and_no_completion_snap(self):
        c=TransformationController();c.reset(M.VTOL)
        approved=[n.local_matrix.copy() for n in c.model.nodes]
        c.request(M.BATTLEDROID);c.update(1.9)
        endpoint=[n.local_matrix.copy() for n in c.model.nodes]
        c.apply_curve(M.VTOL,M.BATTLEDROID,1-1e-6)
        for n,m in zip(c.model.nodes,endpoint):np.testing.assert_allclose(n.local_matrix,m,atol=1e-8)
        c.reset(M.BATTLEDROID);c.request(M.VTOL);c.update(1.9)
        for n,m in zip(c.model.nodes,approved):np.testing.assert_array_equal(n.local_matrix,m)
        self.assertEqual(DURATIONS[(M.VTOL,M.BATTLEDROID)],1.9)

    def test_nozzle_caches_are_current_immediately_after_each_pose(self):
        c=TransformationController()
        for edge in DURATIONS:
            for t in (0,.2,.5,.8,1):
                c.apply_curve(*edge,t)
                for side in ('Left','Right'):
                    engine=c.model.find_node(side+'Engine');marker=c.model.find_node(side+'EngineExhaust')
                    np.testing.assert_allclose(engine.world_matrix,engine.parent.world_matrix@engine.local_matrix,atol=1e-12)
                    np.testing.assert_allclose(marker.world_matrix,engine.world_matrix@marker.local_matrix,atol=1e-12)
                    self.assertAlmostEqual(np.linalg.norm(marker.world_matrix[:3,3]-engine.world_matrix[:3,3]),
                                           np.linalg.norm(marker.local_matrix[:3,3]),places=12)
        c.reset(M.FIGHTER)
        marker=c.model.find_node('LeftEngineExhaust')
        expected=c.model.root_conversion@c.model.find_node('TC167Root').base_matrix@c.model.find_node('LeftEngine').base_matrix@marker.base_matrix
        np.testing.assert_allclose(marker.world_matrix,expected,atol=1e-12)

    def test_exhaust_full_world_transform_and_physics_untouched(self):
        v=PlayerVehicle((23,450,-79));v.rotate(pitch=.2,yaw=1.2,roll=2.1)
        v.velocity[:]=(180,5,-30);v.throttle=.68
        before=(v.position.copy(),v.velocity.copy(),v.orientation.copy(),v.throttle)
        c=TransformationController(v);c.request(M.BATTLEDROID);c.update(3.4)
        c.model.world_matrices(v.model_matrix())
        for side in ('Left','Right'):
            engine=c.model.find_node(side+'Engine');marker=c.model.find_node(side+'EngineExhaust')
            expected=v.model_matrix()@c.model.root_conversion@c.model.find_node('TC167Root').local_matrix@engine.local_matrix@marker.local_matrix
            np.testing.assert_allclose(marker.world_matrix,expected,atol=1e-11)
        for a,b in zip((v.position,v.velocity,v.orientation,v.throttle),before):np.testing.assert_array_equal(a,b)

    def test_viewer_battledroid_hold_allows_inspection(self):
        self.assertAlmostEqual(TransformationTest.SEQUENCE[2][0]-9.4,7.)
        self.assertEqual(TransformationTest.PERIOD,25.8)
