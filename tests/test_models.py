"""CPU hierarchy/accessor tests and mocked GPU ownership; no display required."""
import copy
import json
import math
import struct
from pathlib import Path
import tempfile
import unittest
from unittest.mock import MagicMock,patch
import numpy as np
from engine.asset_paths import asset_path
from engine.gltf_loader import load_glb,read_glb,build_model,Accessors,AssetError
from engine.model import ModelResources,trs
from engine.quaternion import normalize,multiply,slerp,matrix
from game.player_vehicle import PlayerVehicle
from game.model_test import ModelTest
from engine.game import Game
from scripts.generate_hierarchy_test import build

ASSET=asset_path('models/test/hierarchy_test.glb')

class ModelTests(unittest.TestCase):
    def setUp(self):self.model=load_glb(ASSET)

    def test_asset_reproducible_and_no_gl_work_during_load(self):
        self.assertEqual(ASSET.read_bytes(),build())
        with patch('engine.mesh.Mesh',side_effect=AssertionError('Unexpected GPU allocation')):
            m=load_glb(ASSET);self.assertEqual(len(m.nodes),11)

    def test_expected_hierarchy_parent_children_lookup(self):
        m=self.model;leg=m.find_node('LeftLeg');foot=m.find_node('LeftFoot')
        self.assertIs(foot.parent,leg);self.assertIn(foot,leg.children)
        self.assertIs(leg.parent,m.find_node('Root'))
        self.assertIn('    LeftFoot',m.hierarchy())
        with self.assertRaisesRegex(KeyError,'LeftIntake.*not found'):m.find_node('LeftIntake')

    def test_decoded_trs_material_normal_uv_and_mesh_reuse(self):
        m=self.model
        np.testing.assert_allclose(m.find_node('LeftLeg').base_matrix[:3,3],(.7,-.8,0))
        self.assertEqual(m.find_node('LeftLeg').base_trs,((.7,-.8,0.),(0.,0.,0.,1.),(1.,1.,1.)))
        np.testing.assert_allclose(m.find_node('LeftFoot').base_matrix[:3,:3],np.diag((.8,.35,1.2)))
        self.assertEqual(m.find_node('LeftWing').meshes,m.find_node('RightWing').meshes)
        p=m.primitives[0]
        np.testing.assert_allclose(p.positions[0],(-.5,-.5,-.5))
        np.testing.assert_allclose(p.normals[0],(-1,0,0))
        np.testing.assert_allclose(p.texcoords[1],(1,0))
        self.assertEqual(p.indices.dtype,np.dtype('uint16'))
        self.assertEqual(p.material.base_color,(.25,.6,.9,1))
        self.assertEqual(p.material.roughness,.8)
        self.assertFalse(p.positions.flags.writeable)

    def test_parent_articulation_moves_nested_child_not_other_leg(self):
        m=self.model;m.world_matrices();foot=m.find_node('LeftFoot');right=m.find_node('RightFoot')
        before=foot.world_matrix.copy();unchanged=right.world_matrix.copy()
        m.find_node('LeftLeg').set_delta(rotation=(math.sin(math.pi/12),0,0,math.cos(math.pi/12)))
        m.world_matrices();self.assertFalse(np.allclose(before,foot.world_matrix))
        np.testing.assert_allclose(right.world_matrix,unchanged)
        np.testing.assert_allclose(foot.world_matrix,foot.parent.world_matrix@foot.local_matrix)

    def test_child_override_composes_without_corrupting_parent_or_base(self):
        m=self.model;leg=m.find_node('LeftLeg');foot=m.find_node('LeftFoot')
        base=foot.base_matrix;leg.set_delta(rotation=(.2,0,0,.98));m.world_matrices();parent=leg.world_matrix.copy()
        foot.set_delta(rotation=(0,.3,0,.95));m.world_matrices()
        np.testing.assert_allclose(leg.world_matrix,parent)
        np.testing.assert_allclose(foot.base_matrix,base)
        np.testing.assert_allclose(foot.world_matrix,parent@foot.local_matrix)
        public_base=foot.base_matrix;public_base[0,3]=999
        np.testing.assert_allclose(foot.base_matrix,base)

    def test_absolute_pose_override_and_reset_without_reload(self):
        m=self.model;foot=m.find_node('LeftFoot');base=foot.base_matrix
        foot.set_pose(translation=(3,2,1),scale=(2,2,2))
        np.testing.assert_allclose(foot.local_matrix[:3,3],(3,2,1))
        with patch('engine.gltf_loader.read_glb',side_effect=AssertionError('Unexpected reload')):m.reset_pose()
        np.testing.assert_array_equal(foot.local_matrix,base)

    def test_vehicle_root_and_coordinate_conversion_do_not_write_physics(self):
        v=PlayerVehicle((20,30,-40));v.rotate(pitch=.3,yaw=.8,roll=.7);v.velocity[:]=(5,-2,20)
        snapshot=(v.position.copy(),v.velocity.copy(),v.orientation.copy())
        m=self.model;world=v.model_matrix();m.world_matrices(world)
        root=m.find_node('Root')
        np.testing.assert_allclose(root.world_matrix,world@m.root_conversion@root.local_matrix)
        for a,b in zip((v.position,v.velocity,v.orientation),snapshot):np.testing.assert_array_equal(a,b)
        self.assertAlmostEqual(np.linalg.det(m.root_conversion[:3,:3]),1)
        np.testing.assert_allclose(m.root_conversion@np.array((0,0,1,0)),(0,0,-1,0))

    def test_quaternion_xyzw_multiply_slerp_and_sign(self):
        q=(0,math.sin(math.pi/4),0,math.cos(math.pi/4))
        np.testing.assert_allclose(matrix(q)@np.array((0,0,1,0)),(1,0,0,0),atol=1e-12)
        np.testing.assert_allclose(matrix(multiply(q,q))[:3,:3],np.diag((-1,1,-1)),atol=1e-12)
        np.testing.assert_allclose(matrix(slerp((0,0,0,1),multiply(q,q),.5)),matrix(q),atol=1e-12)
        np.testing.assert_allclose(matrix(slerp(q,-np.array(q),.5)),matrix(q))
        with self.assertRaises(ValueError):normalize((0,0,0,0))
        with self.assertRaises(ValueError):slerp(q,q,2)

    def test_node_matrix_column_major_and_quaternion_import(self):
        doc,binary=read_glb(ASSET)
        transform=trs((2,3,4),(0,0,math.sin(.3),math.cos(.3)),(2,3,4))
        doc['nodes'][0]['matrix']=transform.flatten(order='F').tolist()
        m=build_model(doc,binary,convert_coordinates=False)
        np.testing.assert_allclose(m.find_node('Root').base_matrix,transform)
        doc['nodes'][0].pop('matrix');doc['nodes'][0]['rotation']=[0,math.sin(.2),0,math.cos(.2)]
        m=build_model(doc,binary);np.testing.assert_allclose(m.find_node('Root').base_matrix,matrix(doc['nodes'][0]['rotation']))

    def test_bounds_include_hierarchy_units_and_configurable_scale(self):
        low,high=self.model.bounds();self.assertTrue(np.all(low<high))
        m=load_glb(ASSET,scale=2);low2,high2=m.bounds()
        np.testing.assert_allclose(low2,low*2);np.testing.assert_allclose(high2,high*2)
        self.assertAlmostEqual(high[0]-low[0],5.6)

    def test_accessor_all_unsigned_indices_offsets_and_stride(self):
        for dtype,component in (('u1',5121),('<u2',5123),('<u4',5125)):
            array=np.array((2,1,0),dtype=dtype);prefix=b'\0'*8
            doc={'buffers':[{'byteLength':8+array.nbytes}],
                 'bufferViews':[{'buffer':0,'byteOffset':4,'byteLength':4+array.nbytes}],
                 'accessors':[{'bufferView':0,'byteOffset':4,'componentType':component,'count':3,'type':'SCALAR'}]}
            np.testing.assert_array_equal(Accessors(doc,prefix+array.tobytes()).decode(0,'SCALAR',(5121,5123,5125)),array)
        doc,binary=read_glb(ASSET);positions=Accessors(doc,binary).decode(0,'VEC3',(5126,))
        self.assertEqual(positions.shape,(24,3));self.assertEqual(positions.min(),-.5);self.assertEqual(positions.max(),.5)

    def test_normalized_unsigned_uv(self):
        for dtype,component in (('u1',5121),('<u2',5123)):
            array=np.array(((0,np.iinfo(dtype).max),),dtype=dtype)
            doc={'buffers':[{'byteLength':array.nbytes}], 'bufferViews':[{'buffer':0,'byteLength':array.nbytes}],
                 'accessors':[{'bufferView':0,'componentType':component,'normalized':True,'count':1,'type':'VEC2'}]}
            np.testing.assert_allclose(Accessors(doc,array.tobytes()).decode(0,'VEC2',(5121,5123)),((0,1),))

    def test_missing_attribute_unsupported_feature_and_invalid_bounds_errors(self):
        source,binary=read_glb(ASSET)
        edits=[lambda d:d['meshes'][0]['primitives'][0]['attributes'].pop('POSITION'),
               lambda d:d.update(skins=[{}]),lambda d:d.update(animations=[{}]),lambda d:d.update(textures=[{}]),
               lambda d:d.update(extensionsUsed=['KHR_draco_mesh_compression']),
               lambda d:d['accessors'][0].update(sparse={}),
               lambda d:d['accessors'][0].update(count=10000),
               lambda d:d['bufferViews'][0].update(byteStride=8),
               lambda d:d['meshes'][0]['primitives'][0].update(mode=1),
               lambda d:d['nodes'][5]['children'].append(0),
               lambda d:d['nodes'][4].update(children=[7]),
               lambda d:d['materials'][0].update(alphaMode='BLEND')]
        for edit in edits:
            d=copy.deepcopy(source);edit(d)
            with self.assertRaises((AssetError,ValueError)):build_model(d,binary)

    def test_glb_error_includes_filename(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'bad.glb';path.write_bytes(b'bad')
            with self.assertRaisesRegex(AssetError,'bad.glb.*Truncated'):load_glb(path)
            with self.assertRaisesRegex(AssetError,'missing.glb'):load_glb(Path(temp)/'missing.glb')

    def test_gpu_upload_once_per_primitive_draw_instances_close_once(self):
        a,b=MagicMock(),MagicMock()
        with patch('engine.mesh.Mesh',side_effect=[a,b]) as mesh:
            resources=ModelResources(self.model);self.assertEqual(mesh.call_count,2)
            # 8 mesh instances reuse only 2 GPU resources, including on repeated frames.
            shader=MagicMock()
            resources.draw(self.model,shader);resources.draw(self.model,shader)
            self.assertEqual(mesh.call_count,2)
            self.assertEqual(shader.set_matrix.call_count,16)
            self.assertEqual(a.draw.call_count,10);self.assertEqual(b.draw.call_count,6)
            resources.close();resources.close()
            a.close.assert_called_once();b.close.assert_called_once()
            with self.assertRaises(RuntimeError):resources.draw(self.model,shader)

    def test_partial_gpu_failure_releases_uploaded_meshes(self):
        first=MagicMock()
        with patch('engine.mesh.Mesh',side_effect=[first,RuntimeError('upload failure')]):
            with self.assertRaises(RuntimeError):ModelResources(self.model)
        first.close.assert_called_once()

    def test_asset_path_independent_of_cwd(self):
        with tempfile.TemporaryDirectory() as temp:
            import os
            previous=Path.cwd()
            try:
                os.chdir(temp);m=load_glb(asset_path('models/test/hierarchy_test.glb'))
                self.assertEqual(m.find_node('Root').name,'Root')
            finally:os.chdir(previous)

    def test_duplicate_name_is_not_arbitrary_lookup(self):
        self.model.nodes[1].name='Root'
        with self.assertRaisesRegex(ValueError,'ambiguous'):self.model.find_node('Root')

    def test_runtime_proof_parent_child_root_and_reset(self):
        with patch('builtins.print'):proof=ModelTest()
        proof.update(3);self.assertEqual(proof.phase,'PARENT')
        leg=proof.model.find_node('LeftLeg');foot=proof.model.find_node('LeftFoot')
        self.assertFalse(np.allclose(leg.local_matrix,leg.base_matrix))
        proof.update(3);self.assertEqual(proof.phase,'CHILD');self.assertFalse(np.allclose(foot.local_matrix,foot.base_matrix))
        proof.update(3);self.assertEqual(proof.phase,'WORLD ROOT');self.assertFalse(np.allclose(proof.root[:3,:3],np.eye(3)))
        proof.update(4);self.assertEqual(proof.phase,'REST RESET')
        for node in proof.model.nodes:np.testing.assert_array_equal(node.local_matrix,node.base_matrix)

    def test_game_opt_in_proof_never_changes_vehicle_state(self):
        g=Game(enemy_count=0)
        with patch('builtins.print'):g.world.model_test=ModelTest()
        p=g.player_vehicle.position.copy();o=g.player_vehicle.orientation.copy();v=g.player_vehicle.velocity.copy()
        g.update(0)
        np.testing.assert_array_equal(g.player_vehicle.position,p);np.testing.assert_array_equal(g.player_vehicle.velocity,v)
        np.testing.assert_array_equal(g.player_vehicle.orientation,o)

    def test_renderer_owns_optional_resource_and_closes_it_once(self):
        from engine.renderer import Renderer
        g=Game(enemy_count=0)
        renderer=Renderer(g.world,g.player_vehicle,g.camera)
        resource=MagicMock();renderer.model_resources=resource
        renderer.close();renderer.close()
        resource.close.assert_called_once()
        self.assertIsNone(renderer.model_resources)

    def test_malformed_container_and_json_root_have_path_errors(self):
        original=ASSET.read_bytes()
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'broken.glb'
            for data in (b'BAD!'+original[4:],original[:-1]):
                path.write_bytes(data)
                with self.assertRaisesRegex(AssetError,'broken.glb'):load_glb(path)
            encoded=b'[]  '
            path.write_bytes(struct.pack('<4sII',b'glTF',2,24)+struct.pack('<II',4,0x4E4F534A)+encoded)
            with self.assertRaisesRegex(AssetError,'JSON root must be an object'):load_glb(path)

if __name__=='__main__':unittest.main()
