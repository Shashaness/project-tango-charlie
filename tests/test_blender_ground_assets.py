"""The checked-in Blender exports satisfy the runtime asset contracts."""

import runpy
import tempfile
import unittest
from pathlib import Path

from engine.jeep_asset import contract_components
from tools.validate_ground_assets import ROOT, RUNTIME, check_jeep, check_pilot


class AuthoredGroundAssetTests(unittest.TestCase):
    def test_pilot_contract_and_legacy_skeleton_rejected(self):
        check_pilot(RUNTIME["pilot"])
        builder = runpy.run_path(str(ROOT / "scripts/generate_pilot.py"))
        with tempfile.TemporaryDirectory() as directory:
            legacy = Path(directory) / "pilot.glb"
            legacy.write_bytes(builder["build"]())
            with self.assertRaisesRegex(ValueError, "Pilot skin joints differ"):
                check_pilot(legacy)

    def test_anatomical_seat_keeps_knees_forward_and_feet_below_them(self):
        import numpy as np

        asset = check_pilot(RUNTIME["pilot"])
        seat = asset._seated_palette @ np.linalg.inv(asset.inverse)
        positions = {j.name: m[:3, 3] for j, m in zip(asset.joints, seat, strict=True)}
        np.testing.assert_allclose(positions["Hips"], asset.seat_hips)
        for side in ("L", "R"):
            hip, knee, ankle = (positions[f"{n}.{side}"] for n in ("Thigh", "Shin", "Foot"))
            shoulder, elbow, hand = (
                positions[f"{n}.{side}"] for n in ("UpperArm", "Forearm", "Hand")
            )
            self.assertGreater(knee[2] - hip[2], 0.3)
            self.assertGreater(knee[1] - ankle[1], 0.25)
            self.assertGreater(hand[2] - shoulder[2], 0.23)
            self.assertLess(elbow[1], shoulder[1])

    def test_jeep_contract(self):
        model = check_jeep(RUNTIME["jeep"])
        components = contract_components(model)
        if components is None:
            self.assertIs(model.find_node("DriverSeat").parent, model.find_node("Jeep"))
        else:
            anchors, hubs, spins = components
            self.assertIs(anchors["DriverSeat"].parent, model.roots[0])
            self.assertEqual([node.parent for node in spins], hubs)
