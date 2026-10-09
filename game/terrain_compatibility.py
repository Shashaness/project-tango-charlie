"""Vectorized distance to the union of compact infrastructure footprints."""
import numpy as np

class CompatibilityMask:
    def __init__(self, footprints, clearance=150., blend_width=450., variation=.15,
                 wavelength=1800., seed=20, join_smoothing=100.):
        self.footprints=tuple(tuple(float(v) for v in bounds) for bounds in footprints)
        if not self.footprints or not np.isfinite(self.footprints).all() or any(a>=b or c>=d for a,b,c,d in self.footprints):
            raise ValueError('Compatibility footprints must be nonempty rectangles')
        if not np.isfinite([clearance,blend_width,variation,wavelength,join_smoothing]).all() or clearance < 0 or blend_width <= 0 or not 0 <= variation < 1 or wavelength <= 0 or join_smoothing < 0 or not isinstance(seed,(int,np.integer)):
            raise ValueError('Invalid compatibility mask dimensions or seed')
        self.clearance=clearance;self.blend_width=blend_width
        self.variation=variation;self.wavelength=wavelength;self.seed=seed
        self.join_smoothing=join_smoothing

    def distance(self, x, z):
        x,z=np.broadcast_arrays(np.asarray(x,float),np.asarray(z,float))
        distance=np.full(x.shape,np.inf)
        for a,b,c,d in self.footprints:
            dx=np.maximum.reduce((a-x,x-b,np.zeros(x.shape)))
            dz=np.maximum.reduce((c-z,z-d,np.zeros(z.shape)))
            candidate=np.hypot(dx,dz)
            if self.join_smoothing > 0:
                h=np.maximum(self.join_smoothing-np.abs(distance-candidate),0)/self.join_smoothing
                distance=np.minimum(distance,candidate)-h*h*self.join_smoothing/4
            else: distance=np.minimum(distance,candidate)
        return np.maximum(distance,0)

    def contains(self, x, z, strict=False):
        x,z=np.broadcast_arrays(np.asarray(x,float),np.asarray(z,float))
        result=np.zeros(x.shape,dtype=bool)
        for a,b,c,d in self.footprints:
            result |= ((x>a)&(x<b)&(z>c)&(z<d) if strict else
                       (x>=a)&(x<=b)&(z>=c)&(z<=d))
        return result

    def transition_width(self, x, z):
        phase=(self.seed%997)*.173
        frequency=2*np.pi/self.wavelength
        noise=(np.sin(np.asarray(x)*frequency+phase)*np.cos(np.asarray(z)*frequency*.73-phase)
               +.5*np.sin((np.asarray(x)+np.asarray(z))*.43*frequency+phase*.31))/1.5
        return self.blend_width*(1+self.variation*noise)

    def weight(self, x, z):
        t=np.clip((self.distance(x,z)-self.clearance)/self.transition_width(x,z),0,1)
        # Quintic smoothstep: zero first and second derivatives at both ends.
        return t*t*t*(10+t*(-15+6*t))

    def excludes_patch(self, bounds):
        a,b,c,d=bounds
        return any(e<=a and b<=f and g<=c and d<=h for e,f,g,h in self.footprints)
