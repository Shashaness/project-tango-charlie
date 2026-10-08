"""M16.4 elbow-axis direction and symmetric neutral arm regressions."""
import unittest
import numpy as np
from game.transformation import TransformationController
from game.flight_state import VehicleMode as M
from game.tc167_poses import BATTLEDROID_POSE,rotation

class ArmPoseTests(unittest.TestCase):
    def test_actual_hierarchy_forward_flexion_and_symmetric_hands(self):
        c=TransformationController();c.reset(M.BATTLEDROID)
        positions=[]
        for side in ('Left','Right'):
            shoulder,upper,forearm,hand=[c.model.find_node(side+n) for n in ('Shoulder','UpperArm','Forearm','Hand')]
            self.assertIs(upper.parent,shoulder);self.assertIs(forearm.parent,upper);self.assertIs(hand.parent,forearm)
            # Imported links run along -Z with no mirrored joint rotations.
            np.testing.assert_array_equal(upper.base_matrix[:3,:3],np.eye(3))
            np.testing.assert_array_equal(forearm.base_matrix[:3,:3],np.eye(3))
            self.assertEqual(BATTLEDROID_POSE[forearm.name].translation,(0,0,-2))
            self.assertEqual(BATTLEDROID_POSE[hand.name].translation,(0,0,-1.8))
            np.testing.assert_allclose(BATTLEDROID_POSE[forearm.name].rotation,rotation(x=-20),atol=1e-14)
            np.testing.assert_allclose(hand.local_matrix[:3,:3],np.eye(3),atol=1e-14)
            s,e,h=[n.world_matrix[:3,3] for n in (shoulder,forearm,hand)]
            self.assertLess(e[1],s[1]);self.assertAlmostEqual(e[2],s[2])
            self.assertLess(h[1],e[1]);self.assertLess(h[2],e[2]-.5) # engine forward is -Z
            self.assertGreater(abs(h[0]),1.9)
            self.assertAlmostEqual(h[1],2.58,delta=.02) # hips remain at 2.6873
            positions.append(np.array([s,e,h]))
        np.testing.assert_allclose(positions[0]*(-1,1,1),positions[1],atol=1e-12)

    def test_arm_reversal_and_completion_are_continuous(self):
        c=TransformationController();c.reset(M.VTOL)
        stored=[n.local_matrix.copy() for n in c.model.nodes]
        c.request(M.BATTLEDROID);c.update(1.9*.75);c.reverse();c.update(1.9*.1)
        ref=TransformationController();ref.apply_curve(M.VTOL,M.BATTLEDROID,.65)
        for n,r in zip(c.model.nodes,ref.model.nodes):np.testing.assert_allclose(n.local_matrix,r.local_matrix,atol=1e-12)
        c.update(2)
        for n,r in zip(c.model.nodes,stored):np.testing.assert_array_equal(n.local_matrix,r)
        c.reset(M.BATTLEDROID);endpoint=[n.local_matrix.copy() for n in c.model.nodes]
        c.apply_curve(M.VTOL,M.BATTLEDROID,1-1e-6)
        for n,r in zip(c.model.nodes,endpoint):np.testing.assert_allclose(n.local_matrix,r,atol=1e-8)
