"""Milestone 11 tuning; these values never change player flight parameters."""
from dataclasses import dataclass

AI_ENEMY_COUNT = 4
AI_GUNS_ENABLED = True
AI_MISSILES_ENABLED = True

@dataclass(frozen=True)
class AIParameters:
    ecm_enabled: bool = False
    target_speed: float = 160.0
    min_combat_speed: float = 85.0
    high_speed: float = 220.0
    min_altitude: float = 150.0
    terrain_lookahead: float = 5.0
    aggression: float = 1.5
    reaction_time: float = 0.15
    detection_range: float = 12000.0
    pure_pursuit_range: float = 400.0
    firing_tolerance_deg: float = 1.5
    burst_duration: float = 0.6
    burst_pause: float = 0.8
    missile_inventory: int = 4
    missile_cooldown: float = 7.0
    break_period: float = 3.0
    guns_enabled: bool = AI_GUNS_ENABLED
    missiles_enabled: bool = AI_MISSILES_ENABLED

AI = AIParameters()
