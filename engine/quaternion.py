"""Small glTF [x,y,z,w] quaternion utilities; mathematical column-vector matrices."""
import math
import numpy as np


def normalize(value):
    q=np.asarray(value,dtype=np.float64)
    if q.shape!=(4,) or not np.isfinite(q).all() or np.linalg.norm(q)<1e-12:
        raise ValueError('Quaternion must be four finite components with nonzero length.')
    return q/np.linalg.norm(q)


def multiply(left,right):
    a=normalize(left);b=normalize(right)
    return normalize(np.r_[a[3]*b[:3]+b[3]*a[:3]+np.cross(a[:3],b[:3]),
                           a[3]*b[3]-np.dot(a[:3],b[:3])])


def matrix(value):
    x,y,z,w=normalize(value)
    result=np.eye(4)
    result[:3,:3]=((1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)),
                  (2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)),
                  (2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)))
    return result


def slerp(left,right,t):
    if not math.isfinite(t) or not 0<=t<=1:raise ValueError('Interpolation fraction must be in [0,1].')
    a=normalize(left);b=normalize(right);dot=float(np.dot(a,b))
    if dot<0:b=-b;dot=-dot
    if dot>.9995:return normalize(a+t*(b-a))
    angle=math.acos(float(np.clip(dot,-1,1)))
    return normalize((math.sin((1-t)*angle)*a+math.sin(t*angle)*b)/math.sin(angle))
