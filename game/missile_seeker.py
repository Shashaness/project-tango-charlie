"""Assigned-target geometry gate with a finite track-loss grace period."""

from enum import Enum,auto
import math
import numpy as np


class SeekerState(Enum):
    SEPARATING=auto()
    TRACKING=auto()
    SEARCHING=auto()
    LOST=auto()


class MissileSeeker:
    def __init__(self,max_range,half_cone_deg,loss_timeout):
        self.max_range=max_range
        self.half_cone_deg=half_cone_deg
        self.loss_timeout=loss_timeout
        self.lost_time=0.0
        self.state=SeekerState.SEPARATING

    def update(self,relative_position,forward,target_alive,dt):
        distance=float(np.linalg.norm(relative_position))
        visible=(target_alive and math.isfinite(distance) and distance<=self.max_range
                 and (distance<1e-8 or float(np.dot(relative_position/distance,forward))
                      >=math.cos(math.radians(self.half_cone_deg))))
        if visible:
            self.lost_time=0.0
            self.state=SeekerState.TRACKING
        else:
            self.lost_time+=dt
            self.state=SeekerState.LOST if self.lost_time>=self.loss_timeout else SeekerState.SEARCHING
        return visible

class SeekerType(Enum):
    RADAR = auto()
    IR = auto()

IR_SEEKER_RANGE = 6000.0
IR_MIN_HEAT = .15
ECM_SCORE_FACTOR = .55
ECM_LOCK_FACTOR = .5
ECM_DETECTION_FACTOR = 1.5


def heat_signature(target):
    return float(getattr(target,'heat_signature',0.0))


def candidate_score(missile, target, position=None):
    """Signal x angular weighting x inverse-square attenuation. No randomness."""
    if not target.alive: return 0.
    relative = (target.position if position is None else position)-missile.position
    distance = float(np.linalg.norm(relative))
    if not math.isfinite(distance): return 0.
    seeker = missile.seeker
    max_range = min(seeker.max_range,IR_SEEKER_RANGE) if missile.seeker_type is SeekerType.IR else seeker.max_range
    if distance > max_range: return 0.
    alignment = 1. if distance < 1e-8 else float(np.dot(relative/distance,missile.forward))
    cone = math.cos(math.radians(seeker.half_cone_deg))
    if alignment < cone: return 0.
    if missile.seeker_type is SeekerType.IR:
        signal = heat_signature(target)
        if signal < IR_MIN_HEAT: return 0.
    else:
        signal = float(getattr(target,'radar_signature',1.0))
        if getattr(target,'ecm_enabled',False): signal *= ECM_SCORE_FACTOR
    if not math.isfinite(signal) or signal <= 0: return 0.
    angular = .2+.8*(alignment-cone)/max(1e-8,1-cone)
    return signal*angular/(1+(distance/1000)**2)


def select_candidate(missile, decoys, positions=None):
    """Conservative transfer: original aircraft plus matching live decoys only.

    A current valid track gets hysteresis. No other aircraft can be acquired.
    """
    from game.countermeasure import CountermeasureType
    kind = CountermeasureType.FLARE if missile.seeker_type is SeekerType.IR else CountermeasureType.CHAFF
    candidates = [missile.original_target]+[d for d in decoys if d.kind is kind and d.alive]
    scores = {}
    for target in candidates:
        pos = None if positions is None else positions.get(target)
        scores[target] = candidate_score(missile,target,pos)
    missile.seeker.candidate_scores = scores
    current = missile.current_seeker_target
    best = max(candidates,key=lambda t:scores[t])
    if scores.get(current,0)>0 and scores[best] < scores[current]*1.2: best=current
    return best if scores[best]>0 else None
