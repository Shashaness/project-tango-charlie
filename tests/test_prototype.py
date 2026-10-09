"""Prototype structure, mechanical joints and visual-only integration."""
import math
from unittest import TestCase
from unittest.mock import Mock, patch
import numpy as np
from engine.gltf_loader import load_glb
from engine.model import ModelResources
from game.prototype_model import PROTOTYPE_PATH, PrototypeTest, PIVOT_SEQUENCE
from game.player_vehicle import PlayerVehicle
from tools.generate_tc167_prototype import build

class PrototypeTests(TestCase):
    def setUp(self):
        self.model=load_glb(PROTOTYPE_PATH)

    def test_source_reproduces_asset(self):
        from engine.asset_paths import asset_path
        self.assertEqual(build(),asset_path('models/vehicles/tc167_prototype.glb').read_bytes())

    def test_names_and_parent_chain(self):
        expected={'Fuselage':'TC167Root','Nose':'Fuselage','Cockpit':'Fuselage',
                  'Head':'Fuselage','TorsoCore':'Fuselage','VerticalTail':'TC167Root','GunMount':'TC167Root'}
        for side in ('Left','Right'):
            for part in ('Wing','Tail','Intake','Engine','Shoulder'):expected[side+part]='TC167Root'
            for part,parent in [('UpperLeg','Intake'),('LowerLeg','UpperLeg'),('Foot','LowerLeg'),('UpperArm','Shoulder'),('Forearm','UpperArm'),('Hand','Forearm'),('EngineExhaust','Engine')]:expected[side+part]=side+parent
            for number in ('01','02'):expected['Missile'+side+number]=side+'Wing'
        for name,parent in expected.items():
            with self.subTest(name=name):self.assertIs(self.model.find_node(name).parent,self.model.find_node(parent))

    def test_scale_forward_symmetry_and_triangle_budget(self):
        lo,hi=self.model.bounds();size=hi-lo
        self.assertTrue(11<=size[0]<=13);self.assertTrue(4<=size[1]<=5);self.assertTrue(18<=size[2]<=20)
        self.assertAlmostEqual(lo[0],-hi[0])
        self.assertLess(self.model.find_node('Nose').world_matrix[2,3],0)
        self.assertGreater(self.model.find_node('LeftEngineExhaust').world_matrix[2,3],0)
        triangles=sum(len(p.indices)//3 for p in self.model.primitives)
        self.assertGreater(triangles,500);self.assertLess(triangles,20000)
        self.assertTrue(all(np.isfinite(p.normals).all() for p in self.model.primitives))

    def test_each_joint_rotates_geometry_about_stationary_pivot(self):
        for name in PIVOT_SEQUENCE:
            with self.subTest(name=name):
                self.model.reset_pose();self.model.world_matrices()
                node=self.model.find_node(name);before=node.world_matrix.copy()
                visual=next(c for c in node.children if c.meshes)
                primitive=self.model.primitives[visual.meshes[0]]
                p=np.r_[primitive.positions[0],1];start=visual.world_matrix@p
                node.set_delta(rotation=(math.sin(.2),0,0,math.cos(.2)))
                self.model.world_matrices();end=visual.world_matrix@p
                np.testing.assert_allclose(node.world_matrix[:3,3],before[:3,3])
                self.assertFalse(np.allclose(start,end))
                np.testing.assert_allclose(np.linalg.norm(start[:3]-before[:3,3]),np.linalg.norm(end[:3]-before[:3,3]))
                np.testing.assert_allclose(node.parent.local_matrix,node.parent.base_matrix)
                for child in node.children:np.testing.assert_allclose(child.world_matrix,node.world_matrix@child.local_matrix)
        self.model.reset_pose()
        for n in self.model.nodes:np.testing.assert_array_equal(n.local_matrix,n.base_matrix)

    def test_sequence_reset_and_no_physics_writes(self):
        vehicle=PlayerVehicle();position=vehicle.position.copy();orientation=vehicle.orientation.copy() if hasattr(vehicle,'orientation') else vehicle.forward.copy();velocity=vehicle.velocity.copy()
        with patch('builtins.print'):test=PrototypeTest()
        for time,name in [(6,'LeftWing'),(9,'RightWing'),(12,'LeftUpperLeg'),(15,'LeftLowerLeg'),(18,'LeftFoot'),(21,'LeftUpperArm'),(24,'LeftForearm')]:
            test.elapsed=time-.75;test.update(.75);self.assertEqual(test.phase,name)
            self.assertFalse(np.allclose(test.model.find_node(name).local_matrix,test.model.find_node(name).base_matrix))
            test.model.world_matrices(vehicle.model_matrix()@test.root)
        test.elapsed=26;test.update(1)
        for n in test.model.nodes:np.testing.assert_array_equal(n.local_matrix,n.base_matrix)
        np.testing.assert_array_equal(vehicle.position,position);np.testing.assert_array_equal(vehicle.velocity,velocity)
        np.testing.assert_array_equal(vehicle.orientation if hasattr(vehicle,'orientation') else vehicle.forward,orientation)

    def test_gpu_reuse_and_close_once(self):
        with patch('engine.mesh.Mesh') as mesh:
            resources=ModelResources(self.model);count=mesh.call_count
            shader=Mock();resources.draw(self.model,shader);resources.draw(self.model,shader)
            self.assertEqual(mesh.call_count,count)
            resources.close();resources.close()
            # Factory uses one mock instance, total deletes equals unique uploads.
            self.assertEqual(mesh.return_value.close.call_count,count)

    def test_renderer_uses_prototype_only_for_chase_fighter(self):
        from engine.renderer import Renderer
        from engine.camera import Camera
        from game.flight_state import VehicleMode
        world=Mock(cube_positions=[],model_test=None)
        vehicle=PlayerVehicle();camera=Camera()
        renderer=Renderer(world,vehicle,camera)
        renderer.shader=Mock();renderer.grid=Mock();renderer.vehicle_mesh=Mock()
        renderer.vtol_mesh=Mock();renderer.battledroid_mesh=Mock()
        renderer.fighter_resources=Mock();renderer.fighter_model=self.model
        renderer.hud=Mock();renderer._draw_combat_scene=Mock()
        with patch('engine.renderer.GL.glClear'), patch('engine.renderer.GL.glClearColor'):
            renderer.render();renderer.fighter_resources.draw.assert_called_once()
            renderer.vehicle_mesh.draw.assert_not_called()
            vehicle.flight_state.mode=VehicleMode.VTOL;renderer.render()
            renderer.vehicle_mesh.draw.assert_called_once();renderer.vtol_mesh.draw.assert_called_once()
            vehicle.flight_state.mode=VehicleMode.BATTLEDROID;renderer.render()
            renderer.battledroid_mesh.draw.assert_called_once()
            camera.mode='COCKPIT';vehicle.flight_state.mode=VehicleMode.FIGHTER;renderer.render()
            self.assertEqual(renderer.fighter_resources.draw.call_count,1)
        resources=renderer.fighter_resources
        with patch('engine.renderer.GL.glUseProgram'):
            renderer.close();renderer.close()
        resources.close.assert_called_once()

    def test_vtol_chase_offset_preserved(self):
        from engine.camera import Camera
        from game.flight_state import VehicleMode
        vehicle=PlayerVehicle();vehicle.flight_state.mode=VehicleMode.VTOL
        camera=Camera();camera.follow(vehicle,0)
        np.testing.assert_allclose(camera.position,vehicle.position-vehicle.forward*8+vehicle.up*3)
