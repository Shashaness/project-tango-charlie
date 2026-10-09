"""GLFW window lifecycle and the main game loop."""

import math

import glfw

from engine.camera import Camera
from engine.camera_input import CameraInput
from game.player_vehicle import PlayerVehicle
from game.world import World
from game.enemy_aircraft import PlayerTarget
from game.ai_config import AI_ENEMY_COUNT, AI
from game.flight_state import Environment, VehicleMode
from engine.flight_controller import FlightController, MAX_STEP
from game.weapons import CombatSystem
from game.countermeasure import CountermeasureDispenser, CountermeasureType
from game.defensive_avionics import DefensiveAvionics
from game.missile_seeker import SeekerType
from engine.input import Input
from engine.renderer import Renderer
from game.identity import PROJECT_TITLE, PLAYER_UNIT_DESIGNATION
from game.transformation import TransformationController
from game.battledroid_locomotion import BattledroidLocomotionAnimator
from game.battledroid_animation import BattledroidAnimationController


class Game:
    def __init__(self, title=PROJECT_TITLE, width=1280, height=720, enemy_count=AI_ENEMY_COUNT, ai_parameters=AI, locomotion_animation=True, terrain_config=None):
        self.title = title
        self.width = width
        self.height = height
        self.input = Input()
        self.world = World(enemy_count, ai_parameters, terrain_config)
        self.combat = CombatSystem()
        self.player_vehicle = PlayerVehicle()
        self.player_vehicle.unit_designation = PLAYER_UNIT_DESIGNATION
        self.player_vehicle.transformation = TransformationController(self.player_vehicle)
        self.player_vehicle.locomotion = BattledroidLocomotionAnimator(
            self.player_vehicle.transformation, enabled=locomotion_animation)
        self.player_vehicle.animation = BattledroidAnimationController(self.player_vehicle.locomotion)
        self.player_vehicle.defenses = CountermeasureDispenser()
        self.player_vehicle.avionics = DefensiveAvionics()
        self.world.player_target = PlayerTarget(self.player_vehicle)
        self.combat.status = self.world
        self.flight_controller = FlightController(self.player_vehicle)
        self.camera = Camera()
        self.camera_input = CameraInput()
        self.camera.follow(self.player_vehicle, 0.0)
        self.combat.radar.update(self.player_vehicle,self.world.targets,self.combat.gun)
        self.renderer = None
        self.show_axes = False
        self.reset_runway()

    def reset_runway(self):
        """Authoritative development reset; no synthetic touchdown event."""
        from game.world_environment import RUNWAY
        from game.fighter_ground import clearance
        from game.battledroid_physics import GroundContact
        from game.flight_state import FlightStatus
        from game.atmospheric_physics import calculate_forces
        v = self.player_vehicle
        self.flight_controller.reset_atmosphere()
        v.transformation.reset(VehicleMode.FIGHTER)
        v.transformation.duration = 0.
        v.transformation._coordinate = 0.
        v.transformation._direction = 1
        v.forward[:] = RUNWAY.forward;v.right[:] = (1,0,0);v.up[:] = (0,1,0)
        x,z = RUNWAY.spawn_xz
        v.position[:] = (x,clearance(v),z)
        v.velocity[:] = 0;v.acceleration[:] = 0
        v.pitch_rate = v.yaw_rate = v.roll_rate = 0.
        v.throttle = v.engine_throttle = v.thrust_vector = 0.
        v.ground_contact = GroundContact()
        v.flight_state.status = FlightStatus.GROUNDED
        v.locomotion.clear_layer();v.animation.clear_layer()
        enabled = v.locomotion.enabled
        v.locomotion.__init__(v.transformation, enabled=enabled)
        v.animation.__init__(v.locomotion)
        v.aerodynamics = calculate_forces(v)
        # Retain key edge tracking so held F4 triggers only once.
        down = self.input._debug_down.copy()
        self.input.__init__();self.input._debug_down = down
        self.camera_input.drag = (0.,0.)
        self.camera_input.scroll = 0.
        self.camera_input.reset_pressed = self.camera_input.toggle_pressed = False
        self.camera_input._last_cursor = None
        self.flight_controller.fighter_ground.supported = False
        self.flight_controller.vtol_ground.supported = False
        self.camera.mode = self.camera.external_camera_mode = 'CHASE'
        self.camera.reset_orbit();self.camera.snap_to_vehicle();self.camera.follow(v,0.)
        self.combat.missile_fire_control.reset()
        v.defenses.reset();self.world.countermeasures.clear()
        self.world.spawn_enemies(Environment.ATMOSPHERE,v)

    def setup_defense_test(self, seeker_types, distance=1000.):
        """Startup-only reproducible incoming threats; normal launch and flight code.

        Test launchers begin with acquisition already elapsed in valid geometry.
        They continue flying, but do not launch further weapons in this preset.
        """
        from game.enemy_aircraft import EnemyAircraft
        from dataclasses import replace
        import numpy as np
        self.world.targets = [t for t in self.world.targets if not getattr(t,'hostile',False)]
        self.world.enemies = []
        for index, seeker_type in enumerate(seeker_types):
            position = self.player_vehicle.position + np.array((index*60.,0.,distance+index*100.))
            enemy = EnemyAircraft(11+index,self.player_vehicle.flight_state.environment,position,
                replace(self.world.ai_parameters,guns_enabled=False,missiles_enabled=False),index)
            enemy.combat.on_hit=self.world.player_hit
            radar=enemy.combat.radar;control=enemy.combat.missile_fire_control
            radar.update(enemy,[self.world.player_target],enemy.combat.gun)
            control.select(seeker_type)
            control.update(control.parameters.lock_time,enemy,radar)
            missile=control.launch(enemy)
            if missile is None or not control.in_envelope:
                raise RuntimeError('Defense preset requires valid acquisition/launch geometry.')
            enemy.combat.missiles.append(missile)
            self.world.enemies.append(enemy)
            self.world.targets.append(enemy)
        self.player_vehicle.avionics.update(self.player_vehicle,self.world.player_target,
            self.world.enemies,[m for e in self.world.enemies for m in e.combat.missiles])
        self.combat.radar.update(self.player_vehicle,self.world.targets,self.combat.gun)

    def setup_battledroid_test(self, case):
        """Startup-only physical initial conditions for walking/landing/skid tests."""
        from game.battledroid_physics import BATTLEDROID
        from game.flight_state import FlightStatus
        v=self.player_vehicle
        self.flight_controller.reset_atmosphere()
        v.flight_state.mode = VehicleMode.BATTLEDROID
        v.transformation.reset(VehicleMode.BATTLEDROID)
        v.forward[:]=(0,0,-1);v.right[:]=(1,0,0);v.up[:]=(0,1,0)
        v.throttle=v.engine_throttle=0.;v.velocity[:]=0.
        v.position[:]=(0,BATTLEDROID.foot_clearance,0)
        v.flight_state.status=FlightStatus.GROUNDED
        if case=='landing':
            v.position[1]=12.;v.velocity[1]=-2.;v.throttle=v.engine_throttle=.5
            v.flight_state.status=FlightStatus.FLYING
        elif case=='skid':
            v.position[1]=BATTLEDROID.foot_clearance+.5
            v.velocity[:]=(40,-2,0);v.flight_state.status=FlightStatus.FLYING
        self.world.spawn_enemies(Environment.ATMOSPHERE,v)
        self.camera.snap_to_vehicle();self.camera.follow(v,0.)

    def setup_vtol_test(self,case):
        """Startup-only landing/rest/skim conditions using normal VTOL physics."""
        from game.vtol_ground import foot_geometry
        from game.vtol_physics import VTOL
        from game.atmospheric_physics import PARAMETERS
        from game.flight_state import FlightStatus
        v=self.player_vehicle;self.flight_controller.reset_atmosphere()
        from game.battledroid_physics import GroundContact
        v.ground_contact=GroundContact()
        v.flight_state.mode=VehicleMode.VTOL;v.transformation.reset(VehicleMode.VTOL)
        v.forward[:]=(0,0,-1);v.right[:]=(1,0,0);v.up[:]=(0,1,0)
        v.throttle=v.engine_throttle=0.;v.thrust_vector=1.;v.velocity[:]=0.
        v.position[:]=(0,foot_geometry()[1],0);v.flight_state.status=FlightStatus.GROUNDED
        if case=='landing':
            v.position[1]+=3.;v.velocity[1]=-1.
            v.throttle=v.engine_throttle=PARAMETERS.mass*PARAMETERS.gravity/VTOL.max_thrust
            v.flight_state.status=FlightStatus.FLYING
        elif case in ('hard','crash'):
            v.position[1]+=.05;v.velocity[1]=-8. if case=='hard' else -25.
            v.flight_state.status=FlightStatus.FLYING
        elif case=='skim':v.velocity[0]=8.
        self.world.spawn_enemies(Environment.ATMOSPHERE,v)
        self.camera.snap_to_vehicle();self.camera.follow(v,0.)

    def run(self):
        window = None
        renderer = None
        try:
            if not glfw.init():
                raise RuntimeError("Could not initialize GLFW.")

            glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
            glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
            glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
            # macOS requires a forward-compatible core context.
            glfw.window_hint(glfw.OPENGL_FORWARD_COMPAT, glfw.TRUE)
            window = glfw.create_window(
                self.width, self.height, self.title, None, None
            )
            if not window:
                raise RuntimeError("Could not create an OpenGL 3.3 Core window.")

            glfw.make_context_current(window)
            self.camera_input.install(window)
            glfw.swap_interval(1)
            renderer = Renderer(self.world, self.player_vehicle, self.camera, self.flight_controller.fcc, self.combat)
            self.renderer = renderer
            renderer.initialize()
            glfw.set_framebuffer_size_callback(window, renderer.resize)
            # Framebuffer pixels can differ from window size on Retina displays.
            renderer.resize(window, *glfw.get_framebuffer_size(window))

            print("Flight: W/S pitch down/up, A/D roll left/right, Q/E yaw left/right, "
                  "Up/Down throttle, Left/Right strafe, R/F up/down in SPACE, X space brake, F1 SPACE, F2 ATMOSPHERE, "
                  "R Fighter reset / VTOL up, G cycle Fighter/VTOL/Battledroid, K Battledroid jump, 0 chase/dolly, Z/X VTOL vector, C camera, H debug, F3 SAS, F4 runway reset, V hover assist, TAB next target, M missile, 1/2 RADAR/IR, L flare, B chaff, J ECM, SPACEBAR gun, ESC quit")
            next_title_time = 0.0
            last_time = glfw.get_time()
            while not glfw.window_should_close(window):
                current_time = glfw.get_time()
                dt = current_time - last_time
                last_time = current_time
                glfw.poll_events()
                self.input.poll(window)
                self.camera_input.poll(window)
                if glfw.window_should_close(window):
                    break
                self.update(dt)
                if current_time >= next_title_time:
                    glfw.set_window_title(window, f"{self.title} | {self.camera.label}" + (f" | MODEL {self.world.model_test.phase}" if getattr(self.world,"model_test",None) is not None else ""))
                    next_title_time = current_time + 0.25
                renderer.render()
                glfw.swap_buffers(window)
        finally:
            try:
                if renderer is not None:
                    renderer.close()
            finally:
                if window is not None:
                    glfw.destroy_window(window)
                glfw.terminate()

    def update(self, dt):
        """Advance direct flight controls using elapsed seconds."""
        if self.input.runway_pressed:
            self.reset_runway()
            return
        if self.input.toggle_camera:
            self.camera.toggle_mode()
        self.camera_input.apply(self.camera)
        if self.input.toggle_axes:
            self.show_axes = not self.show_axes
        if self.input.toggle_configuration:
            self.player_vehicle.transformation.cycle()
        reset_view = False
        if self.input.space_pressed:
            self.flight_controller.switch_space()
            self.world.spawn_enemies(Environment.SPACE, self.player_vehicle)
        elif self.input.atmosphere_pressed or (self.input.reset_pressed and
                self.player_vehicle.flight_state.environment is Environment.ATMOSPHERE and
                self.player_vehicle.flight_state.mode is VehicleMode.FIGHTER):
            self.flight_controller.reset_atmosphere()
            self.player_vehicle.transformation.reset(VehicleMode.FIGHTER)
            self.combat.missile_fire_control.reset()
            self.player_vehicle.defenses.reset()
            self.world.countermeasures.clear()
            self.world.spawn_enemies(Environment.ATMOSPHERE, self.player_vehicle)
            reset_view = True
        if self.input.select_radar: self.combat.missile_fire_control.select(SeekerType.RADAR)
        if self.input.select_ir: self.combat.missile_fire_control.select(SeekerType.IR)
        if self.input.toggle_ecm:
            self.player_vehicle.defenses.ecm_enabled = not self.player_vehicle.defenses.ecm_enabled
        if self.input.dispense_flare:
            self.player_vehicle.defenses.dispense(self.player_vehicle,CountermeasureType.FLARE,self.world.countermeasures)
        if self.input.dispense_chaff:
            self.player_vehicle.defenses.dispense(self.player_vehicle,CountermeasureType.CHAFF,self.world.countermeasures)
        if reset_view:
            self.camera.snap_to_vehicle()
        if self.input.toggle_sas:
            self.flight_controller.fcc.toggle_stability()
        if self.input.toggle_hover:
            self.flight_controller.fcc.toggle_hover(self.player_vehicle)
        if not self.flight_controller.fcc.hover_eligible(self.player_vehicle):
            self.flight_controller.fcc.clear_hover()
        if self.input.select_target:
            self.combat.radar.select_next(self.world.targets)
        if not math.isfinite(dt):
            raise ValueError("Delta time must be finite.")
        if dt > 0:
            # Match gun/target motion to flight substeps, including shot birth times.
            steps = max(1, math.ceil(dt / MAX_STEP))
            step = dt / steps
            for substep in range(steps):
                previous_position = self.player_vehicle.position.copy()
                previous_velocity = self.player_vehicle.velocity.copy()
                self.player_vehicle.animation.clear_layer()
                self.player_vehicle.locomotion.clear_layer()
                self.player_vehicle.transformation.update(step)
                if not self.flight_controller.fcc.hover_eligible(self.player_vehicle):
                    self.flight_controller.fcc.clear_hover()
                self.flight_controller.update(step, self.input)
                self.player_vehicle.animation.prepare(step, self.player_vehicle, self.input)
                self.player_vehicle.locomotion.update(step, self.player_vehicle, self.input)
                self.player_vehicle.animation.update()
                target_starts = {target: target.position.copy() for target in self.world.targets}
                candidate_starts = {c:c.position.copy() for c in self.world.countermeasures}
                for package in self.world.countermeasures: package.update(step)
                self.world.countermeasures[:] = [c for c in self.world.countermeasures if c.alive]
                commands = [enemy.fly(step, self.world.player_target, self.combat.missiles,
                                      self.input.fire_gun,self.world.countermeasures) for enemy in self.world.enemies]
                for target in self.world.targets:
                    if not getattr(target, 'hostile', False): target.update(step)
                self.combat.update(step,self.player_vehicle,self.world.targets,self.input.fire_gun,
                                   previous_position,previous_velocity,
                                   launch_missile=self.input.launch_missile and substep==0,
                                   target_starts=target_starts, advance_targets=False,
                                   countermeasures=self.world.countermeasures,candidate_starts=candidate_starts)
                self.world.player_hit_flash = max(0., self.world.player_hit_flash-step)
                for enemy, command in zip(self.world.enemies, commands):
                    live = enemy.alive and command is not None
                    inventory = enemy.combat.missile_fire_control.inventory
                    enemy.combat.update(step, enemy, [self.world.player_target],
                        firing=live and command.fire_gun,
                        previous_position=enemy.previous_position,
                        previous_velocity=enemy.previous_velocity,
                        launch_missile=live and command.fire_missile,
                        target_starts={self.world.player_target: previous_position},
                        advance_targets=False,countermeasures=self.world.countermeasures,
                        candidate_starts=candidate_starts)
                    if enemy.combat.missile_fire_control.inventory < inventory:
                        enemy.pilot.missile_launched()
        else:
            self.flight_controller.update(dt, self.input)
            self.player_vehicle.animation.clear_layer()
            self.player_vehicle.animation.prepare(max(0.,dt), self.player_vehicle, self.input)
            self.player_vehicle.locomotion.update(max(0.,dt), self.player_vehicle, self.input)
            self.player_vehicle.animation.update()
            self.combat.radar.update(self.player_vehicle,self.world.targets,self.combat.gun)
        self.player_vehicle.avionics.update(self.player_vehicle,self.world.player_target,
            self.world.enemies,[m for e in self.world.enemies for m in e.combat.missiles])
        model_test=getattr(self.world,"model_test",None)
        if model_test is not None:model_test.update(dt)
        self.camera.follow(self.player_vehicle, dt)
        if self.renderer is not None:
            self.renderer.show_axes = self.show_axes
