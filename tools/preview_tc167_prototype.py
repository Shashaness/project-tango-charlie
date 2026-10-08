"""CPU-only PNG inspection sheet; development tool, not a game renderer."""
import math
import struct
import zlib
from pathlib import Path
import numpy as np
from engine.gltf_loader import load_glb
from game.prototype_model import PROTOTYPE_PATH


def preview(output, node=None, degrees=20, model=None):
    model=load_glb(PROTOTYPE_PATH) if model is None else model
    if node:
        angle=math.radians(degrees)
        model.find_node(node).set_delta(rotation=(math.sin(angle/2),0,0,math.cos(angle/2)))
    panels=[('TOP',(0,1,0),(0,0,-1)),('SIDE',(1,0,0),(0,1,0)),
            ('FRONT',(0,0,-1),(0,1,0)),('REAR',(0,0,1),(0,1,0)),
            ('BOTTOM',(0,-1,0),(0,0,-1)),('THREE QUARTER',(1,.7,-1),(0,1,0))]
    width,height=960,960;canvas=np.full((height,width,3),(9,18,27),dtype=np.uint8)
    for panel,(name,direction,up) in enumerate(panels):
        d=np.array(direction,dtype=float);d/=np.linalg.norm(d)
        r=np.cross(up,d);r/=np.linalg.norm(r);u=np.cross(d,r)
        basis=np.array([r,u,d]);depth=np.full((320,480),-np.inf)
        offset_x=(panel%2)*480;offset_y=(panel//2)*320
        for index,world in model.world_matrices():
            primitive=model.primitives[index]
            positions=(world@np.column_stack((primitive.positions,np.ones(len(primitive.positions)))).T).T[:,:3]
            projected=positions@basis.T
            projected[:,:2]*=14;projected[:,0]+=240;projected[:,1]=160-projected[:,1]
            for tri in primitive.indices.reshape(-1,3):
                p=projected[tri];x0=max(0,int(np.floor(p[:,0].min())));x1=min(479,int(np.ceil(p[:,0].max())))
                y0=max(0,int(np.floor(p[:,1].min())));y1=min(319,int(np.ceil(p[:,1].max())))
                if x1<x0 or y1<y0:continue
                a,b,c=p;den=(b[1]-c[1])*(a[0]-c[0])+(c[0]-b[0])*(a[1]-c[1])
                if abs(den)<1e-9:continue
                yy,xx=np.mgrid[y0:y1+1,x0:x1+1]
                w0=((b[1]-c[1])*(xx-c[0])+(c[0]-b[0])*(yy-c[1]))/den
                w1=((c[1]-a[1])*(xx-c[0])+(a[0]-c[0])*(yy-c[1]))/den;w2=1-w0-w1
                z=w0*a[2]+w1*b[2]+w2*c[2];region=depth[y0:y1+1,x0:x1+1]
                mask=(w0>=0)&(w1>=0)&(w2>=0)&(z>region)
                normal=np.cross(positions[tri[1]]-positions[tri[0]],positions[tri[2]]-positions[tri[0]])
                normal/=max(np.linalg.norm(normal),1e-9)
                shade=.55+.45*abs(np.dot(normal,np.array([.3,.8,-.5])/np.linalg.norm([.3,.8,-.5])))
                color=np.array(primitive.material.base_color[:3])*255*shade
                region[mask]=z[mask]
                canvas[offset_y+y0:offset_y+y1+1,offset_x+x0:offset_x+x1+1][mask]=color.astype(np.uint8)
        print(panel+1,name)
    def chunk(kind,data):return struct.pack('>I',len(data))+kind+data+struct.pack('>I',zlib.crc32(kind+data)&0xffffffff)
    raw=b''.join(b'\0'+row.tobytes() for row in canvas)
    Path(output).write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',width,height,8,2,0,0,0))+chunk(b'IDAT',zlib.compress(raw))+chunk(b'IEND',b''))

if __name__=='__main__':
    import sys
    preview(sys.argv[1] if len(sys.argv)>1 else '/tmp/tc167_prototype_views.png',
            sys.argv[2] if len(sys.argv)>2 else None,
            float(sys.argv[3]) if len(sys.argv)>3 else 20)
