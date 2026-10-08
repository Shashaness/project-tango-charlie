"""Visual-only Fighter asset and isolated automatic pivot inspection."""
import math
import numpy as np
from engine.asset_paths import asset_path
from engine.gltf_loader import load_glb
from engine.model import trs

PROTOTYPE_PATH = asset_path('models/vehicles/tc167.glb')
PIVOT_SEQUENCE = ('LeftWing', 'RightWing', 'LeftUpperLeg', 'LeftLowerLeg',
                  'LeftFoot', 'LeftUpperArm', 'LeftForearm')

class PrototypeTest:
    def __init__(self):
        self.model = load_glb(PROTOTYPE_PATH)
        self.model.print_hierarchy()
        low, high = self.model.bounds()
        print('Prototype dimensions X/Y/Z (meters):', high-low)
        self.elapsed = 0.
        self.phase = 'REST'
        self.root = trs((14, 0, -18))

    def update(self, dt):
        self.elapsed += max(0., dt)
        cycle = self.elapsed % 29
        self.model.reset_pose()
        # Rotate the whole model during the first five seconds; no physics writes.
        angle = cycle/5*2*math.pi if cycle < 5 else 0.
        self.root = trs((14,0,-18), rotation=(0,math.sin(angle/2),0,math.cos(angle/2)))
        if cycle < 5:
            self.phase = 'WHOLE MODEL'
        elif cycle < 26:
            index = int((cycle-5)//3)
            self.phase = PIVOT_SEQUENCE[index]
            angle = math.radians(20)*math.sin((cycle-5)%3/3*2*math.pi)
            self.model.find_node(self.phase).set_delta(rotation=(math.sin(angle/2),0,0,math.cos(angle/2)))
        else:
            self.phase = 'REST RESET'
