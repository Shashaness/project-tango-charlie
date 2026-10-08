"""Gun/projectile lifecycle and moving-target swept hits; no vehicle physics writes."""

import math
import numpy as np

from game.fire_control_radar import FireControlRadar
from game.gun import Gun
from game.missile_fire_control import MissileFireControl
from game.projectile import segment_sphere_hit

HIT_FLASH_SECONDS = .6


class CombatSystem:
    def __init__(self, on_hit=None):
        self.on_hit = on_hit
        self.kills = 0
        self.status = None
        self.gun = Gun()
        self.radar = FireControlRadar()
        self.projectiles = []
        self.missile_fire_control = MissileFireControl()
        self.missiles = []
        self.miss_flash = 0.0
        self.hit_flash = 0.0

    def update(self, dt, vehicle, targets, firing=False, previous_position=None, previous_velocity=None, launch_missile=False, target_starts=None, advance_targets=True, countermeasures=(), candidate_starts=None):
        if not math.isfinite(dt) or dt < 0:
            raise ValueError('Combat timestep must be finite and nonnegative.')
        self.miss_flash = max(0.0,self.miss_flash-dt)
        self.hit_flash = max(0.0,self.hit_flash-dt)
        if target_starts is None:
            target_starts = {target: target.position.copy() for target in targets}
        if advance_targets:
            for target in targets:
                target.update(dt)
        events = [(projectile,0.0) for projectile in self.projectiles if projectile.alive]
        events.extend(self.gun.update(dt,firing,vehicle,previous_position,previous_velocity))
        survivors = []
        for projectile,birth in events:
            travel_time = projectile.update(max(0.0,dt-birth))
            earliest_hit = None
            if dt>0 and travel_time>0:
                for target in targets:
                    if not target.alive:
                        continue
                    motion = target.position-target_starts[target]
                    start = target_starts[target] + motion*(birth/dt)
                    end = start + motion*(travel_time/dt)
                    # Relative swept motion makes fast crossing targets collide correctly.
                    fraction = segment_sphere_hit(projectile.previous_position-start,
                                                  projectile.position-end,np.zeros(3),
                                                  projectile.radius+target.radius)
                    if fraction is not None and (earliest_hit is None or fraction<earliest_hit[0]):
                        earliest_hit = (fraction,target)
            if earliest_hit is not None:
                self.hit_target(earliest_hit[1], "GUN")
                projectile.alive = False
                self.hit_flash = HIT_FLASH_SECONDS
            if projectile.alive:
                survivors.append(projectile)
        self.projectiles = survivors
        active=[]
        for missile in self.missiles:
            start=target_starts.get(missile.target,missile.target.position.copy())
            previous_age=missile.age
            travel_time=missile.update(dt,start,countermeasures,candidate_starts)
            if travel_time>0 and missile.age>=missile.parameters.separation_time:
                armed_delay=max(0.0,missile.parameters.separation_time-previous_age)
                projectile_start=missile.previous_position+(missile.position-missile.previous_position)*(armed_delay/travel_time)
                fuse_hit=None
                for target in targets:
                    if not target.alive:continue
                    start=target_starts[target]
                    motion = target.position-start
                    armed_start=start+motion*(armed_delay/dt)
                    target_end=start+motion*(travel_time/dt)
                    fraction=segment_sphere_hit(projectile_start-armed_start,
                                               missile.position-target_end,np.zeros(3),
                                               missile.parameters.fuse_radius+target.radius)
                    if fraction is not None and (fuse_hit is None or fraction<fuse_hit[0]):
                        fuse_hit=(fraction,target)
                # Decoys are sensed/fused by missiles only; never gun/radar targets.
                tracked=missile.current_seeker_target
                if tracked in countermeasures and tracked.alive:
                    start=tracked.position if candidate_starts is None else candidate_starts.get(tracked,tracked.position)
                    motion=tracked.position-start
                    fraction=segment_sphere_hit(projectile_start-(start+motion*(armed_delay/dt)),
                                               missile.position-(start+motion*(travel_time/dt)),np.zeros(3),
                                               missile.parameters.fuse_radius+tracked.radius)
                    if fraction is not None and (fuse_hit is None or fraction<fuse_hit[0]):
                        fuse_hit=(fraction,tracked)
                if fuse_hit is not None:
                    if fuse_hit[1] in countermeasures:
                        fuse_hit[1].alive=False
                    else:
                        self.hit_target(fuse_hit[1], "MISSILE")
                        self.hit_flash=HIT_FLASH_SECONDS
                    missile.alive=False;missile.miss_reason=None
            if missile.alive:active.append(missile)
            elif missile.miss_reason is not None:self.miss_flash=HIT_FLASH_SECONDS
        self.missiles=active
        self.radar.update(vehicle,targets,self.gun)
        self.missile_fire_control.update(dt,vehicle,self.radar)
        if launch_missile:
            missile=self.missile_fire_control.launch(vehicle)
            if missile is not None:self.missiles.append(missile)

    def hit_target(self, target, weapon):
        if not target.alive:
            return
        if self.on_hit is not None:
            self.on_hit(target, weapon)
        else:
            target.alive = False
            if getattr(target, 'hostile', False):
                self.kills += 1
                # Keep destruction instrumentation immediate, including zero-dt queries.
                from game.ai_pilot import AIState
                target.pilot.state = AIState.DEAD
