"""Opt-in hierarchy proof alongside primitive vehicles; never writes simulation state."""
import math
import numpy as np
from engine.asset_paths import asset_path
from engine.gltf_loader import load_glb
from engine.model import trs

class ModelTest:
    def __init__(self,path=None):
        self.model=load_glb(asset_path('models/test/hierarchy_test.glb') if path is None else path)
        self.model.print_hierarchy()
        print('Vehicle-local bounds:',self.model.bounds())
        self.elapsed=0.;self.phase='REST';self.root=np.eye(4)
        self.rest_only=path is not None  # arbitrary imported assets have no assumed node names

    def update(self,dt):
        self.elapsed+=max(0.,dt)
        cycle=self.elapsed%16
        self.model.reset_pose()
        self.root=trs((7,0,-12))
        if self.rest_only:return
        leg=self.model.find_node('LeftLeg');foot=self.model.find_node('LeftFoot')
        angle=math.radians(30)
        def rotation(angle):return (math.sin(angle/2),0,0,math.cos(angle/2))
        if cycle<4:
            self.phase='PARENT';leg.set_delta(rotation=rotation(angle*cycle/4))
        elif cycle<8:
            self.phase='CHILD';leg.set_delta(rotation=rotation(angle));foot.set_delta(rotation=rotation(angle*(cycle-4)/4))
        elif cycle<12:
            self.phase='WORLD ROOT';leg.set_delta(rotation=rotation(angle));foot.set_delta(rotation=rotation(angle))
            self.root=trs((7+math.sin(cycle-8)*2,0,-12),rotation=(0,math.sin((cycle-8)*.2),0,math.cos((cycle-8)*.2)))
        else:self.phase='REST RESET'
