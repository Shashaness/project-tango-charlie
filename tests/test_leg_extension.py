"""Native-length Battledroid stance and late continuous deployment."""
import unittest
import numpy as np
from game.transformation import TransformationController
from game.tc167_poses import BATTLEDROID_POSE, rotation
from game.flight_state import VehicleMode as M

class LegExtensionTests(unittest.TestCase):
    def test_extended_native_links_level_feet_and_contact(self):
        c=TransformationController();c.reset(M.BATTLEDROID)
        lo,hi=c.model.bounds()
        self.assertAlmostEqual(lo[1],-3.2,delta=.001)
        self.assertGreater(hi[1]-lo[1],10.7)
        for sign,side in ((1,'Left'),(-1,'Right')):
            self.assertEqual(BATTLEDROID_POSE[side+'Intake'].translation,(sign*1.1,-.6,-.4))
            joints=[c.model.find_node(side+n) for n in ('UpperLeg','LowerLeg','Foot')]
            for node in joints:self.assertEqual(BATTLEDROID_POSE[node.name].scale,(1,1,1))
            for a,b in zip(joints,joints[1:]):
                self.assertAlmostEqual(np.linalg.norm(b.world_matrix[:3,3]-a.world_matrix[:3,3]),2.6,places=6)
                self.assertGreater(a.world_matrix[1,3]-b.world_matrix[1,3],2.59)
            np.testing.assert_allclose(joints[-1].world_matrix[:3,1],(0,-1,0),atol=1e-12)
            self.assertTrue(.45<(joints[0].world_matrix[1,3]-lo[1])/(hi[1]-lo[1])<.55)
            for node,angle in zip(joints,(-94,8,-94)):
                np.testing.assert_allclose(BATTLEDROID_POSE[node.name].rotation,rotation(x=angle),atol=1e-14)

    def test_intake_shell_retains_compression_without_compressing_links(self):
        c=TransformationController();c.reset(M.BATTLEDROID)
        for side in ('Left','Right'):
            intake=c.model.find_node(side+'Intake')
            self.assertEqual(BATTLEDROID_POSE[intake.name].scale,(.85,1,1))
            for node in intake.children:
                if 'Surface' in node.name:
                    np.testing.assert_allclose(np.linalg.norm(node.world_matrix[:3,:3],axis=0),(.85,.448,1),atol=1e-12)

    def test_leg_extension_is_late_and_reverse_retraces(self):
        c=TransformationController();c.reset(M.VTOL)
        original=c.model.find_node('LeftUpperLeg').local_matrix.copy()
        c.apply_curve(M.VTOL,M.BATTLEDROID,.34)
        np.testing.assert_allclose(c.model.find_node('LeftUpperLeg').local_matrix,original,atol=1e-14)
        c.reset(M.VTOL);c.request(M.BATTLEDROID);c.update(1.9*.7)
        c.reverse();c.update(1.9*.1)
        reference=TransformationController();reference.apply_curve(M.VTOL,M.BATTLEDROID,.6)
        for a,b in zip(c.model.nodes,reference.model.nodes):
            np.testing.assert_allclose(a.local_matrix,b.local_matrix,atol=1e-12)
