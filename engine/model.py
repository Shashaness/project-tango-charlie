"""CPU hierarchy/rest data and context-owned GPU resources, separate from physics."""
from dataclasses import dataclass
import numpy as np
from engine.quaternion import matrix as quaternion_matrix

GLTF_TO_VEHICLE=np.diag((-1.,1.,-1.,1.))  # proper 180-degree Y rotation, not reflection


def checked_matrix(value):
    value=np.array(value,dtype=np.float64,copy=True)
    if value.shape!=(4,4) or not np.isfinite(value).all() or not np.allclose(value[3],(0,0,0,1)):
        raise ValueError('Transform must be a finite affine 4x4 matrix.')
    return value


def trs(translation=(0,0,0),rotation=(0,0,0,1),scale=(1,1,1)):
    t=np.asarray(translation,dtype=float);s=np.asarray(scale,dtype=float)
    if t.shape!=(3,) or s.shape!=(3,) or not np.isfinite(t).all() or not np.isfinite(s).all():
        raise ValueError('Translation/scale must have three finite components.')
    result=quaternion_matrix(rotation);result[:3,:3]=result[:3,:3]@np.diag(s);result[:3,3]=t
    return result

@dataclass(frozen=True)
class Material:
    name: str = 'Default'
    base_color: tuple = (1.,1.,1.,1.)
    metallic: float = 1.
    roughness: float = 1.
    double_sided: bool = False

@dataclass(frozen=True)
class MeshPrimitive:
    positions: np.ndarray
    normals: np.ndarray | None
    texcoords: np.ndarray | None
    indices: np.ndarray | None
    material: Material
    name: str

class ModelNode:
    def __init__(self,name,base_matrix=None,meshes=(),base_trs=None):
        self.name=name;self.parent=None;self.children=[];self.meshes=tuple(meshes)
        # Preserve imported TRS components for later pose interpolation; matrix
        # nodes keep their exact matrix and expose no invented decomposition.
        self.base_trs=None if base_trs is None else tuple(tuple(float(x) for x in v) for v in base_trs)
        self._base=checked_matrix(np.eye(4) if base_matrix is None else base_matrix)
        self._base.setflags(write=False)
        self._override=None;self._delta=np.eye(4)
        self.world_matrix=np.eye(4)

    @property
    def base_matrix(self):
        return self._base.copy()

    @property
    def local_matrix(self):
        return (self._base if self._override is None else self._override)@self._delta

    def set_pose(self,translation=(0,0,0),rotation=(0,0,0,1),scale=(1,1,1)):
        """Absolute runtime TRS override, leaving the imported matrix intact."""
        self._override=trs(translation,rotation,scale);self._delta=np.eye(4)

    def set_delta(self,translation=(0,0,0),rotation=(0,0,0,1),scale=(1,1,1)):
        """Rest-relative local-space articulation: effective = base * delta."""
        self._override=None;self._delta=trs(translation,rotation,scale)

    def set_animation_delta(self, matrix=None):
        """Local visual layer after the current configuration pose; no rest writes."""
        self._delta=np.eye(4) if matrix is None else checked_matrix(matrix)

    def reset_pose(self):
        self._override=None;self._delta=np.eye(4)

class Model:
    def __init__(self,nodes,roots,primitives,scale=1.,convert_coordinates=True):
        if not np.isfinite(scale) or scale<=0:raise ValueError('Model scale must be positive and finite.')
        self.nodes=list(nodes);self.roots=list(roots);self.primitives=tuple(primitives)
        self.root_conversion=GLTF_TO_VEHICLE.copy() if convert_coordinates else np.eye(4)
        self.root_conversion[:3,:3]*=scale

    def find_node(self,name):
        matches=[n for n in self.nodes if n.name==name]
        if len(matches)==1:return matches[0]
        if matches:raise ValueError(f'Model node {name!r} is ambiguous ({len(matches)} matches).')
        raise KeyError(f'Model node {name!r} not found; available: '+', '.join(n.name for n in self.nodes))

    def reset_pose(self):
        for node in self.nodes:node.reset_pose()

    def world_matrices(self,world=None):
        parent=checked_matrix(np.eye(4) if world is None else world)@self.root_conversion
        draws=[]
        def visit(node,parent):
            node.world_matrix=parent@node.local_matrix
            if not np.isfinite(node.world_matrix).all():raise ValueError(f"Nonfinite world transform for {node.name!r}.")
            for mesh in node.meshes:draws.append((mesh,node.world_matrix.copy()))
            for child in node.children:visit(child,node.world_matrix)
        for root in self.roots:visit(root,parent)
        return draws

    def bounds(self,world=None):
        points=[]
        for index,matrix in self.world_matrices(world):
            p=self.primitives[index].positions
            points.append((matrix@np.column_stack((p,np.ones(len(p)))).T).T[:,:3])
        if not points:raise ValueError('Selected scene contains no geometry.')
        points=np.concatenate(points)
        if not np.isfinite(points).all():raise ValueError("Model bounds contain nonfinite vertices.")
        return points.min(axis=0),points.max(axis=0)

    def hierarchy(self):
        lines=[]
        def visit(node,depth):
            mesh_names=', '.join(self.primitives[i].name for i in node.meshes)
            lines.append('  '*depth+node.name+(f' [{mesh_names}]' if mesh_names else ''))
            for child in node.children:visit(child,depth+1)
        for root in self.roots:visit(root,0)
        return '\n'.join(lines)

    def print_hierarchy(self):
        print(self.hierarchy())

class ModelResources:
    """Unique primitive uploads reused across nodes/models; renderer owns close().

    Construct/draw/close only on the render thread with a current GL context.
    No GL work happens while loading Model CPU data.
    """
    def __init__(self,model):
        from engine.mesh import Mesh
        self.meshes=[];self._closed=False
        try:
            for primitive in model.primitives:
                colors=np.tile(primitive.material.base_color[:3],(len(primitive.positions),1))
                self.meshes.append(Mesh(np.column_stack((primitive.positions,colors)),primitive.indices))
        except Exception:
            self.close();raise

    def draw(self,model,shader,world=None):
        if self._closed:raise RuntimeError('Model GPU resources have been closed.')
        for index,matrix in model.world_matrices(world):
            shader.set_matrix('model',matrix);self.meshes[index].draw()

    def close(self):
        if self._closed:return
        for mesh in self.meshes:mesh.close()
        self.meshes.clear();self._closed=True
