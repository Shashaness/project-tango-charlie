"""Keyboard-free repeating M16 visual inspection; no simulation state writes."""
import numpy as np
from engine.model import trs
from game.transformation import TransformationController
from game.flight_state import VehicleMode

class TransformationTest:
    # Absolute seconds: endpoint holds and both forward/reverse staged curves.
    SEQUENCE = ((3.,VehicleMode.VTOL),(7.5,VehicleMode.BATTLEDROID),
                (16.4,VehicleMode.VTOL),(21.3,VehicleMode.FIGHTER))
    PERIOD = 25.8

    def __init__(self):
        self.controller=TransformationController()
        self.model=self.controller.model
        self.model.print_hierarchy()
        self.elapsed=0.;self.phase='FIGHTER';self.root=trs((14,0,-18))
        self._event=0;self._reported=None

    def update(self,dt):
        if not np.isfinite(dt) or dt<0:raise ValueError('Inspection dt must be finite and nonnegative')
        remaining=dt
        while remaining>0:
            boundary=self.SEQUENCE[self._event][0] if self._event<len(self.SEQUENCE) else self.PERIOD
            step=min(remaining,max(0.,boundary-self.elapsed))
            self.controller.update(step);self.elapsed+=step;remaining-=step
            if self.elapsed>=boundary-1e-10:
                if self._event<len(self.SEQUENCE):
                    self.controller.request(self.SEQUENCE[self._event][1]);self._event+=1
                else:
                    self.controller.reset();self.elapsed=0.;self._event=0
        self.phase=self.controller.label
        state=(self.controller.source,self.controller.target,self.controller.active,self.controller.configuration)
        if state!=self._reported:
            print('MODEL TEST:',self.phase);self._reported=state
