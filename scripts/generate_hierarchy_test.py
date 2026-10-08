"""Reproducible original technical GLB; standard library only, no Blender/downloads."""
import json
from pathlib import Path
import struct

OUTPUT=Path(__file__).resolve().parent.parent/'assets/models/test/hierarchy_test.glb'


def build():
    corners=[(x,y,z) for x in (-.5,.5) for y in (-.5,.5) for z in (-.5,.5)]
    faces=((0,1,3,2),(4,6,7,5),(0,4,5,1),(2,3,7,6),(0,2,6,4),(1,5,7,3))
    normals=((-1,0,0),(1,0,0),(0,-1,0),(0,1,0),(0,0,-1),(0,0,1))
    vertices=[];indices=[]
    for face,normal in zip(faces,normals):
        base=len(vertices)
        for index,uv in zip(face,((0,0),(1,0),(1,1),(0,1))):vertices.append((*corners[index],*normal,*uv))
        indices.extend(base+i for i in (0,1,2,0,2,3))
    # Deliberate view and accessor offsets, with 32-byte interleaved stride.
    vertex_data=b'\0'*4+b''.join(struct.pack('<8f',*v) for v in vertices)
    index_data=b'\0'*2+struct.pack('<36H',*indices)
    binary=b'\0'*4+vertex_data+index_data
    doc={'asset':{'version':'2.0','generator':'Project Tango Charlie hierarchy test generator'},
         'scene':0,'scenes':[{'name':'HierarchyTest','nodes':[0]}],
         'buffers':[{'byteLength':len(binary)}],
         'bufferViews':[{'buffer':0,'byteOffset':4,'byteLength':len(vertex_data),'byteStride':32,'target':34962},
                        {'buffer':0,'byteOffset':4+len(vertex_data),'byteLength':len(index_data),'target':34963}],
         'accessors':[{'bufferView':0,'byteOffset':4,'componentType':5126,'count':24,'type':'VEC3','min':[-.5]*3,'max':[.5]*3},
                      {'bufferView':0,'byteOffset':16,'componentType':5126,'count':24,'type':'VEC3'},
                      {'bufferView':0,'byteOffset':28,'componentType':5126,'count':24,'type':'VEC2'},
                      {'bufferView':1,'byteOffset':2,'componentType':5123,'count':36,'type':'SCALAR'}],
         'materials':[{'name':'BodyBlue','pbrMetallicRoughness':{'baseColorFactor':[.25,.6,.9,1],'metallicFactor':0,'roughnessFactor':.8}},
                      {'name':'WingOrange','pbrMetallicRoughness':{'baseColorFactor':[1,.5,.15,1],'metallicFactor':.2,'roughnessFactor':.6}}],
         'meshes':[{'name':name,'primitives':[{'attributes':{'POSITION':0,'NORMAL':1,'TEXCOORD_0':2},'indices':3,'material':i}]} for i,name in enumerate(('SharedBodyBox','SharedOrangeBox'))],
         'nodes':[{'name':'Root','children':[1,2,3,4,5,8]},
                  {'name':'Body','mesh':0,'scale':[2,1.2,3]},
                  {'name':'Nose','mesh':1,'translation':[0,0,2],'scale':[.7,.5,1]},
                  {'name':'LeftWing','mesh':1,'translation':[1.9,0,0],'scale':[1.8,.2,1.1]},
                  {'name':'RightWing','mesh':1,'translation':[-1.9,0,0],'scale':[1.8,.2,1.1]},
                  {'name':'LeftLeg','translation':[.7,-.8,0],'rotation':[0,0,0,1],'children':[6,7]},
                  {'name':'LeftLegShape','mesh':0,'translation':[0,-.7,0],'scale':[.5,1.4,.5]},
                  {'name':'LeftFoot','mesh':0,'translation':[0,-1.5,.3],'scale':[.8,.35,1.2]},
                  {'name':'RightLeg','translation':[-.7,-.8,0],'children':[9,10]},
                  {'name':'RightLegShape','mesh':0,'translation':[0,-.7,0],'scale':[.5,1.4,.5]},
                  {'name':'RightFoot','mesh':0,'translation':[0,-1.5,.3],'scale':[.8,.35,1.2]}]}
    encoded=json.dumps(doc,separators=(',',':')).encode();encoded+=b' '*(-len(encoded)%4)
    binary+=b'\0'*(-len(binary)%4)
    return struct.pack('<4sII',b'glTF',2,12+8+len(encoded)+8+len(binary))+struct.pack('<II',len(encoded),0x4E4F534A)+encoded+struct.pack('<II',len(binary),0x004E4942)+binary

if __name__=='__main__':
    OUTPUT.parent.mkdir(parents=True,exist_ok=True);OUTPUT.write_bytes(build());print(OUTPUT)
