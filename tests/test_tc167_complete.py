"""M21.2 exported transitions agree with the production procedural controller."""

import unittest

import numpy as np

from engine.asset_paths import asset_path
from engine.gltf_animation import AnimationPlayer
from engine.gltf_loader import load_glb, read_glb
from game.flight_state import VehicleMode as Mode
from game.transformation import TransformationController

EDGES = {
    "TC_FighterToVTOL": (Mode.FIGHTER, Mode.VTOL, False, 1.5),
    "TC_VTOLToFighter": (Mode.FIGHTER, Mode.VTOL, True, 1.5),
    "TC_VTOLToBattledroid": (Mode.VTOL, Mode.BATTLEDROID, False, 1.9),
    "TC_BattledroidToVTOL": (Mode.VTOL, Mode.BATTLEDROID, True, 1.9),
}


class CompleteAssetTests(unittest.TestCase):
    def setUp(self):
        self.model = load_glb(asset_path("models/vehicles/tc167_complete.glb"))
        self.player = AnimationPlayer(self.model)
        self.controller = TransformationController()

    def test_rest_hierarchy_geometry_materials_unchanged(self):
        original, binary = read_glb(asset_path("models/vehicles/tc167.glb"))
        complete, data = read_glb(asset_path("models/vehicles/tc167_complete.glb"))
        for field in ("nodes", "meshes", "materials", "scenes", "scene"):
            self.assertEqual(original.get(field), complete.get(field))
        self.assertEqual(binary, data[: len(binary)])

    def test_clip_inventory_and_locomotion_preserved(self):
        old = load_glb(asset_path("models/vehicles/tc167_animated.glb"))
        self.assertEqual(
            set(self.model.animations),
            set(EDGES)
            | set(old.animations)
            | {"TC_Pose_Fighter", "TC_Pose_VTOL", "TC_Pose_Battledroid"},
        )
        for name, clip in old.animations.items():
            new = self.model.animations[name]
            self.assertAlmostEqual(new.duration, clip.duration, places=6)
            for t in np.linspace(0, clip.duration, 101):
                a = {(n.name, p): v for (n, p), v in clip.sample(float(t)).items()}
                b = {(n.name, p): v for (n, p), v in new.sample(float(t)).items()}
                self.assertEqual(a.keys(), b.keys())
                for key in a:
                    if key[1] == "rotation" and np.dot(a[key], b[key]) < 0:
                        b[key] = -b[key]
                    np.testing.assert_allclose(a[key], b[key], atol=2e-6)

    def test_transition_endpoints_and_interiors(self):
        for name, (first, last, reverse, duration) in EDGES.items():
            clip = self.model.animations[name]
            self.assertAlmostEqual(clip.duration, duration, places=6)
            self.assertEqual(
                {c.node.name for c in clip.channels}, {n.name for n in self.model.nodes}
            )
            self.assertEqual(len(clip.channels), 204)
            self.player.play(name, loop=False)
            for t in np.linspace(0, 1, 121):
                self.controller.apply_curve(first, last, 1 - t if reverse else t)
                self.player.time = float(t * clip.duration)
                self.player.apply()
                for a, b in zip(self.model.nodes, self.controller.model.nodes):
                    tolerance = 2e-5 if t in (0, 1) else 0.004
                    np.testing.assert_allclose(
                        a.local_matrix,
                        b.local_matrix,
                        atol=tolerance,
                        err_msg=f"{name} {t} {a.name}",
                    )
                    self.assertTrue(np.isfinite(a.world_matrix).all())
                before = [n.local_matrix.copy() for n in self.model.nodes]
                self.player.apply()
                for node, matrix in zip(self.model.nodes, before):
                    np.testing.assert_array_equal(node.local_matrix, matrix)

    def test_reverse_and_clip_seams(self):
        for forward, backward in (
            ("TC_FighterToVTOL", "TC_VTOLToFighter"),
            ("TC_VTOLToBattledroid", "TC_BattledroidToVTOL"),
        ):
            a, b = self.model.animations[forward], self.model.animations[backward]
            for u in np.linspace(0, 1, 41):
                left, right = a.sample(float(u * a.duration)), b.sample(float((1 - u) * b.duration))
                for key in left:
                    x, y = left[key], right[key]
                    if key[1] == "rotation" and np.dot(x, y) < 0:
                        y = -y
                    np.testing.assert_allclose(x, y, atol=2e-5)
        a = self.model.animations["TC_FighterToVTOL"]
        b = self.model.animations["TC_VTOLToBattledroid"]
        for key, value in a.sample(a.duration).items():
            np.testing.assert_allclose(value, b.sample(0)[key], atol=2e-6)

    def test_game_physics_camera_and_transform_ownership_match_existing_asset(self):
        from engine.game import Game

        for case in ("flight", "ground", "vtol", "battledroid", "transform"):
            games = [
                Game(legacy_animation=True, enemy_count=0, vehicle_model_path=asset_path("models/vehicles/" + name))
                for name in ("tc167_animated.glb", "tc167_complete.glb")
            ]
            for game in games:
                if case == "battledroid":
                    game.setup_battledroid_test("ground")
                elif case == "flight":
                    from game.flight_state import FlightStatus

                    game.player_vehicle.flight_state.status = FlightStatus.FLYING
                    game.player_vehicle.position[1] = 1000
                    game.player_vehicle.velocity[:] = (0, 0, -150)
                elif case == "vtol":
                    game.setup_vtol_test("rest")
                elif case == "transform":
                    game.player_vehicle.transformation.request(Mode.BATTLEDROID)
                game.input.pitch = -0.3
                game.input.roll = 0.2
                game.input.fire_gun = True
            for _ in range(30):
                for game in games:
                    game.update(1 / 60)
            a, b = [g.player_vehicle for g in games]
            for field in ("position", "velocity", "orientation", "acceleration"):
                np.testing.assert_array_equal(getattr(a, field), getattr(b, field))
            np.testing.assert_array_equal(games[0].camera.position, games[1].camera.position)
            self.assertEqual(games[0].combat.gun.cooldown, games[1].combat.gun.cooldown)
            self.assertEqual(a.animation.native_clip, b.animation.native_clip)
            if b.transformation.active:
                self.assertIsNone(b.animation.native_clip)
                for node in b.transformation.model.nodes:
                    np.testing.assert_array_equal(node.local_matrix, node.configuration_matrix)
            for game in games:
                game.world.terrain.dataset.close()
