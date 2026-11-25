"""GLFW window lifecycle and the main game loop."""

import math

import glfw

from engine.camera import Camera
from engine.camera_input import CameraInput
from engine.driving_camera import DrivingCamera
from engine.flight_controller import MAX_STEP, FlightController
from engine.helmet_information import HelmetInformation
from engine.input import Input
from engine.pedestrian_camera import PedestrianCamera
from engine.renderer import Renderer
from game.ai_config import AI, AI_ENEMY_COUNT
from game.battledroid_animation import BattledroidAnimationController
from game.battledroid_locomotion import BattledroidLocomotionAnimator
from game.countermeasure import CountermeasureDispenser, CountermeasureType
from game.defensive_avionics import DefensiveAvionics
from game.enemy_aircraft import PlayerTarget
from game.flight_state import Environment, FlightStatus, VehicleMode
from game.identity import PLAYER_UNIT_DESIGNATION, PROJECT_TITLE
from game.missile_seeker import SeekerType
from game.player_control import ControlMode, PlayerControl, VehicleAccess
from game.player_vehicle import PlayerVehicle
from game.transformation import TransformationController
from game.weapons import CombatSystem
from game.world import World


class Game:
    """Own the GLFW window and coordinate input, simulation, rendering, and cleanup."""

    def __init__(
        self,
        title=PROJECT_TITLE,
        width=1280,
        height=720,
        enemy_count=AI_ENEMY_COUNT,
        ai_parameters=AI,
        locomotion_animation=True,
        terrain_config=None,
        vehicle_model_path=None,
        gltf_animation=True,
        legacy_animation=False,
        animation_debug=False,
        anchor_debug=False,
        world_manifest=None,
        campaign=None,
    ):
        """Create the window, shared game systems, and initial development spawn.

        Args:
            title: Window title displayed by GLFW.
            width: Viewport width in pixels.
            height: Viewport height in pixels.
            enemy_count: Number of hostile aircraft to spawn.
            ai_parameters: Immutable AI tuning parameters.
            locomotion_animation: Whether the procedural battledroid animation layer is
                enabled.
            terrain_config: Optional terrain source/LOD configuration.
            vehicle_model_path: Optional TC167 GLB retaining the existing named hierarchy.
            gltf_animation: Whether imported battledroid clips may replace procedural gait.
        """
        self.title = title
        self.width = width
        self.height = height
        self.input = Input()
        self.helmet_information = HelmetInformation()
        self.campaign = campaign
        if campaign is not None:
            world_manifest, enemy_count = campaign.definition, 0
        self.world = World(enemy_count, ai_parameters, terrain_config, world_manifest)
        self.combat = CombatSystem()
        self.player_vehicle = PlayerVehicle()
        self.player_vehicle.identity = self.world.aircraft_spawn.identity
        self.world.live_entities[self.player_vehicle.identity] = self.player_vehicle
        self.player_vehicle.geographic_frame = self.world.terrain.frame
        self.player_vehicle.unit_designation = PLAYER_UNIT_DESIGNATION
        self.player_vehicle.animation_debug = animation_debug
        self.player_vehicle.anchor_debug = anchor_debug
        model = None
        if vehicle_model_path is None and not legacy_animation:
            vehicle_model_path = self.world.aircraft_spawn.definition.asset.path
        if vehicle_model_path is not None:
            from engine.gltf_loader import load_glb

            model = load_glb(vehicle_model_path)
        controller = TransformationController
        if not legacy_animation:
            from game.authored_transformation import AuthoredTransformationController

            controller = AuthoredTransformationController
        self.player_vehicle.transformation = controller(self.player_vehicle, model=model)
        self.player_vehicle.locomotion = BattledroidLocomotionAnimator(
            self.player_vehicle.transformation, enabled=locomotion_animation
        )
        self.player_vehicle.animation = BattledroidAnimationController(
            self.player_vehicle.locomotion, native_enabled=gltf_animation
        )
        self.player_vehicle.defenses = CountermeasureDispenser()
        self.player_vehicle.avionics = DefensiveAvionics()
        self.world.player_target = PlayerTarget(self.player_vehicle)
        self.combat.status = self.world
        self.flight_controller = FlightController(self.player_vehicle, self.world.ground)
        self.camera = Camera()
        self.pedestrian_camera = PedestrianCamera()
        self.player_control = PlayerControl(self.player_vehicle, self.world.ground)
        for vehicle in self.world.ground_vehicles:
            self.player_control.register(VehicleAccess(vehicle))
        self.world.player_control = self.player_control
        self.driving_camera = DrivingCamera()
        self._parked_input = Input()
        self.camera_input = CameraInput()
        self.camera.follow(self.player_vehicle, 0.0)
        self.combat.radar.update(self.player_vehicle, self.world.targets, self.combat.gun)
        self.renderer = None
        self.show_axes = False
        if self.campaign is None:
            self.reset_runway()
        self._spawn_authored_aircraft()
        self.initialize_player()
        from game.navigation import Navigation

        self.navigation = Navigation(
            self.player_control, self.world.scene.definition.identity, self.campaign
        )
        if self.campaign is not None:
            self.campaign.bind(self)

    def spawn_hangar(self):
        """Reset the player to the hangar development spawn and snap the camera."""
        if self.campaign is None:
            self._spawn_authored_aircraft()

    def _spawn_authored_aircraft(self):
        import numpy as np

        from game.battledroid_physics import contact_clearance as battledroid_clearance
        from game.fighter_ground import clearance
        from game.vtol_ground import contact_clearance as vtol_clearance

        v = self.player_vehicle
        placement = self.world.aircraft_spawn
        state = placement.definition.initial_state
        mode = VehicleMode[state["configuration"]]
        v.flight_state.mode = mode
        v.flight_state.environment = Environment[state["environment"]]
        v.flight_state.status = FlightStatus[state["status"]]
        if mode is VehicleMode.VTOL:
            v.thrust_vector = 1.0
        if mode is not VehicleMode.FIGHTER or self.campaign is not None:
            v.airfield.reset(deployed=False)
        v.transformation.reset(mode)
        v.forward[:] = placement.forward
        v.up[:] = (0, 1, 0)
        v.right[:] = np.cross(v.forward, v.up)
        v.position[:] = placement.position
        support = self.world.ground.height(v.position)
        ground_clearance = {
            VehicleMode.BATTLEDROID: battledroid_clearance,
            VehicleMode.VTOL: vtol_clearance,
            VehicleMode.FIGHTER: clearance,
        }
        if self.campaign is None:
            v.position[1] = support + ground_clearance[mode](v)
        else:
            lower, _ = self.world.ground.body_bounds(v)
            v.position[1] = support - lower[1]
        self.camera.snap_to_vehicle()
        self.camera.follow(v, 0.0)

    def initialize_player(self):
        from game.player_control import Pilot

        scene = self.world.scene.definition
        control = self.player_control
        spawn = self.world.pilot_spawn
        pilot = Pilot(spawn.position, spawn.forward)
        pilot.position[1] = self.world.ground.height(pilot.position)
        control._pilot = pilot
        self.world.live_entities[spawn.identity] = pilot
        control.mode = ControlMode[scene.player_mode]
        if control.mode is ControlMode.ON_FOOT:
            control.aircraft.deactivate(control.space)
            control.pilot = pilot
            self.pedestrian_camera.begin(pilot)
        elif control.mode is ControlMode.GROUND_VEHICLE:
            control.aircraft.deactivate(control.space)
            control.selected = control.registry[scene.controlled_entity]
            control.selected.activate(pilot, control.space)
            self.driving_camera.begin(control.selected.vehicle)
        else:
            control.aircraft.activate(pilot, control.space)

    def reset_runway(self):
        """Authoritative development reset; no synthetic touchdown event."""
        if self.campaign is not None:
            return
        from game.atmospheric_physics import calculate_forces
        from game.battledroid_physics import GroundContact
        from game.fighter_ground import clearance

        runway = self.world.environment.runway

        v = self.player_vehicle
        self.flight_controller.reset_atmosphere()
        v.transformation.reset(VehicleMode.FIGHTER)
        v.transformation.duration = 0.0
        v.transformation._coordinate = 0.0
        v.transformation._direction = 1
        v.forward[:] = runway.forward
        v.right[:] = (1, 0, 0)
        v.up[:] = (0, 1, 0)
        v.airfield.reset(deployed=True)
        x, z = runway.spawn_xz
        v.position[:] = (x, clearance(v), z)
        v.velocity[:] = 0
        v.acceleration[:] = 0
        v.pitch_rate = v.yaw_rate = v.roll_rate = 0.0
        v.throttle = v.engine_throttle = v.thrust_vector = 0.0
        v.ground_contact = GroundContact()
        v.flight_state.status = FlightStatus.GROUNDED
        v.locomotion.clear_layer()
        v.animation.clear_layer()
        enabled = v.locomotion.enabled
        v.locomotion.__init__(v.transformation, enabled=enabled)
        native_enabled = v.animation.native_enabled
        v.animation.__init__(v.locomotion, native_enabled=native_enabled)
        v.aerodynamics = calculate_forces(v)
        # Retain key edge tracking so held F4 triggers only once.
        down = self.input._debug_down.copy()
        self.input.__init__()
        self.input._debug_down = down
        self.camera_input.drag = (0.0, 0.0)
        self.camera_input.scroll = 0.0
        self.camera_input.reset_pressed = self.camera_input.toggle_pressed = False
        self.camera_input._last_cursor = None
        self.flight_controller.fighter_ground.supported = False
        self.flight_controller.vtol_ground.supported = False
        self.camera.mode = self.camera.external_camera_mode = "CHASE"
        self.camera.reset_orbit()
        self.camera.snap_to_vehicle()
        self.camera.follow(v, 0.0)
        self.combat.missile_fire_control.reset()
        v.defenses.reset()
        self.world.countermeasures.clear()
        self.world.spawn_enemies(Environment.ATMOSPHERE, v)

    def setup_defense_test(self, seeker_types, distance=1000.0):
        """Startup-only reproducible incoming threats; normal launch and flight code.

        Test launchers begin with acquisition already elapsed in valid geometry.
        They continue flying, but do not launch further weapons in this preset.

        Args:
            seeker_types: Sequence of radar/infrared seeker types for incoming test missiles.
            distance: Initial threat spacing in meters behind the player.
        """
        if self.campaign is not None:
            return
        from dataclasses import replace

        import numpy as np

        from game.enemy_aircraft import EnemyAircraft

        self.world.targets = [t for t in self.world.targets if not getattr(t, "hostile", False)]
        self.world.enemies = []
        for index, seeker_type in enumerate(seeker_types):
            position = self.player_vehicle.position + np.array(
                (index * 60.0, 0.0, distance + index * 100.0)
            )
            enemy = EnemyAircraft(
                11 + index,
                self.player_vehicle.flight_state.environment,
                position,
                replace(self.world.ai_parameters, guns_enabled=False, missiles_enabled=False),
                index,
            )
            enemy.combat.on_hit = self.world.player_hit
            radar = enemy.combat.radar
            control = enemy.combat.missile_fire_control
            radar.update(enemy, [self.world.player_target], enemy.combat.gun)
            control.select(seeker_type)
            control.update(control.parameters.lock_time, enemy, radar)
            missile = control.launch(enemy)
            if missile is None or not control.in_envelope:
                raise RuntimeError("Defense preset requires valid acquisition/launch geometry.")
            enemy.combat.missiles.append(missile)
            self.world.enemies.append(enemy)
            self.world.targets.append(enemy)
        self.player_vehicle.avionics.update(
            self.player_vehicle,
            self.world.player_target,
            self.world.enemies,
            [m for e in self.world.enemies for m in e.combat.missiles],
        )
        self.combat.radar.update(self.player_vehicle, self.world.targets, self.combat.gun)

    def setup_battledroid_test(self, case):
        """Startup-only physical initial conditions for walking/landing/skid tests.

        Args:
            case: Named development scenario to initialize.
        """
        if self.campaign is not None:
            return
        from game.flight_state import FlightStatus

        v = self.player_vehicle
        self.flight_controller.reset_atmosphere()
        v.flight_state.mode = VehicleMode.BATTLEDROID
        v.transformation.reset(VehicleMode.BATTLEDROID)
        v.forward[:] = (0, 0, -1)
        v.right[:] = (1, 0, 0)
        v.up[:] = (0, 1, 0)
        v.throttle = v.engine_throttle = 0.0
        v.velocity[:] = 0.0
        from game.battledroid_physics import contact_clearance

        v.position[:] = (0, contact_clearance(v), 0)
        v.flight_state.status = FlightStatus.GROUNDED
        if case == "landing":
            v.position[1] = 12.0
            v.velocity[1] = -2.0
            v.throttle = v.engine_throttle = 0.5
            v.flight_state.status = FlightStatus.FLYING
        elif case == "skid":
            v.position[1] = contact_clearance(v) + 0.5
            # Exercise sliding on the clear airfield rather than through new city colliders.
            v.position[2] = 2500.0
            v.velocity[:] = (40, -2, 0)
            v.flight_state.status = FlightStatus.FLYING
        self.world.spawn_enemies(Environment.ATMOSPHERE, v)
        self.camera.snap_to_vehicle()
        self.camera.follow(v, 0.0)

    def setup_vtol_test(self, case):
        """Startup-only landing/rest/skim conditions using normal VTOL physics.

        Args:
            case: Named development scenario to initialize.
        """
        if self.campaign is not None:
            return
        from game.atmospheric_physics import PARAMETERS
        from game.flight_state import FlightStatus
        from game.vtol_ground import contact_clearance
        from game.vtol_physics import VTOL

        v = self.player_vehicle
        self.flight_controller.reset_atmosphere()
        from game.battledroid_physics import GroundContact

        v.ground_contact = GroundContact()
        v.flight_state.mode = VehicleMode.VTOL
        v.transformation.reset(VehicleMode.VTOL)
        v.forward[:] = (0, 0, -1)
        v.right[:] = (1, 0, 0)
        v.up[:] = (0, 1, 0)
        v.throttle = v.engine_throttle = 0.0
        v.thrust_vector = 1.0
        v.velocity[:] = 0.0
        v.position[:] = (0, contact_clearance(v), 0)
        v.flight_state.status = FlightStatus.GROUNDED
        if case == "landing":
            v.position[1] += 3.0
            v.velocity[1] = -1.0
            v.throttle = v.engine_throttle = PARAMETERS.mass * PARAMETERS.gravity / VTOL.max_thrust
            v.flight_state.status = FlightStatus.FLYING
        elif case in ("hard", "crash"):
            v.position[1] += 0.05
            v.velocity[1] = -8.0 if case == "hard" else -25.0
            v.flight_state.status = FlightStatus.FLYING
        elif case == "skim":
            v.velocity[0] = 8.0
        self.world.spawn_enemies(Environment.ATMOSPHERE, v)
        self.camera.snap_to_vehicle()
        self.camera.follow(v, 0.0)

    def run(self):
        """Run the frame loop, updating state and drawing until the window closes."""
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
            window = glfw.create_window(self.width, self.height, self.title, None, None)
            if not window:
                raise RuntimeError("Could not create an OpenGL 3.3 Core window.")

            glfw.make_context_current(window)
            self.camera_input.install(window)
            glfw.swap_interval(1)
            renderer = Renderer(
                self.world,
                self.player_vehicle,
                self.camera,
                self.flight_controller.fcc,
                self.combat,
                campaign_hud=self.campaign.hud_lines if self.campaign is not None else None,
                object_scene=getattr(self.campaign, "objects", None),
                navigation=self.navigation.snapshot,
                information=lambda: (
                    self.helmet_information.content(self.player_control, self.campaign),
                    self.helmet_information.page,
                    self.helmet_information.banner(),
                    self.helmet_information.context_banner(),
                ),
            )
            self.renderer = renderer
            renderer.initialize()
            glfw.set_framebuffer_size_callback(window, renderer.resize)
            # Framebuffer pixels can differ from window size on Retina displays.
            renderer.resize(window, *glfw.get_framebuffer_size(window))

            print(
                "Flight: W/S pitch down/up, A/D roll left/right, Q/E yaw left/right, "
                "Up/Down throttle, Left/Right strafe, R/F up/down in SPACE, X space brake, "
                "F1 SPACE, F2 ATMOSPHERE, "
                "R Fighter reset / VTOL up, G cycle Fighter/VTOL/Battledroid, K Battledroid jump, "
                "0 chase/dolly, Z/X VTOL vector, C camera, H debug, F3 SAS, F4 runway reset, "
                "6 wheel brakes, 7 flaps, 8 gear, V hover assist, TAB next target, M missile, "
                "1/2 RADAR/IR, L flare, B chaff, J ECM, SPACEBAR gun, ESC quit. "
                "Enter exit/re-enter. On foot: WASD, Shift run, Space jump, "
                "drag orbit, scroll zoom. Jeep: WASD drive, Space brake, P parking brake."
            )
            if self.campaign is not None and self.campaign.saved is None:
                from game.campaign import OPENING

                print("\n".join(OPENING))
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
                    glfw.set_window_title(
                        window,
                        f"{self.title} | {renderer.camera.label}"
                        + (
                            f" | {self.player_control.message}"
                            if self.player_control.message
                            else ""
                        )
                        + (
                            f" | MODEL {self.world.model_test.phase}"
                            if getattr(self.world, "model_test", None) is not None
                            else ""
                        ),
                    )
                    next_title_time = current_time + 0.25
                renderer.render()
                glfw.swap_buffers(window)
        finally:
            try:
                if renderer is not None:
                    try:
                        if self.campaign is not None:
                            self.campaign.save(self)
                    finally:
                        renderer.close()
            finally:
                if window is not None:
                    glfw.destroy_window(window)
                glfw.terminate()

    def update(self, dt):
        """Advance the active player controller and shared simulation.

        Args:
            dt: Elapsed simulation time in seconds.
        """
        if not math.isfinite(dt):
            raise ValueError("Delta time must be finite.")
        self.navigation.elapsed += max(dt, 0)
        controls = self.player_control
        previous_mode = controls.mode
        previous_access = controls.active if self.campaign is not None else None
        previous_drive_position = (
            previous_access.vehicle.position.copy() if previous_access is not None else None
        )
        switched = self.input.interact_pressed and controls.interact()
        cameras = {
            ControlMode.AIRCRAFT: self.camera,
            ControlMode.ON_FOOT: self.pedestrian_camera,
            ControlMode.GROUND_VEHICLE: self.driving_camera,
        }
        camera = cameras[controls.mode]
        if switched:
            camera.aspect = cameras[previous_mode].aspect
            if controls.mode is ControlMode.ON_FOOT:
                camera.begin(controls.pilot)
            elif controls.mode is ControlMode.GROUND_VEHICLE:
                camera.begin(controls.active.vehicle)
            else:
                camera.snap_to_vehicle()
        if not self.input.interact_pressed and (
            self.input.pilot_advance or self.input.pilot_strafe
        ):
            controls.message = ""
        on_foot = controls.mode is ControlMode.ON_FOOT
        driving = controls.mode is ControlMode.GROUND_VEHICLE
        parked = self.campaign is not None or controls.mode is not ControlMode.AIRCRAFT or switched
        commands_input = self._parked_input if parked else self.input
        if self.renderer is not None:
            self.renderer.camera = camera
        if on_foot or driving:
            self.camera_input.apply(camera)
        if on_foot:
            direction = camera.movement_direction(self.input.pilot_strafe, self.input.pilot_advance)
            if not switched:
                controls.pilot.update(
                    dt, controls.space, direction, self.input.pilot_run, self.input.pilot_jump
                )
            camera.observe(controls.pilot, controls.space, dt)
        elif driving:
            vehicle = controls.active.vehicle
            if not switched:
                if self.input.toggle_parking_brake:
                    vehicle.parking_brake = not vehicle.parking_brake
                vehicle.update(
                    dt,
                    controls.space,
                    self.input.pilot_advance,
                    self.input.pilot_strafe,
                    self.input.fire_gun,
                )
            camera.observe(vehicle, controls.space, dt)
        if commands_input.runway_pressed:
            self.reset_runway()
            return
        if commands_input.toggle_camera:
            self.camera.toggle_mode()
        if not on_foot and not driving:
            self.camera_input.apply(self.camera)
        if (
            self.player_vehicle.flight_state.mode is VehicleMode.FIGHTER
            and self.player_vehicle.flight_state.environment is Environment.ATMOSPHERE
        ):
            if commands_input.toggle_gear:
                self.player_vehicle.airfield.toggle_gear()
            if commands_input.toggle_flaps:
                self.player_vehicle.airfield.toggle_flaps()
        if commands_input.toggle_axes:
            self.show_axes = not self.show_axes
        if commands_input.toggle_configuration:
            self.player_vehicle.transformation.cycle()
        reset_view = False
        if commands_input.space_pressed:
            self.flight_controller.switch_space()
            self.world.spawn_enemies(Environment.SPACE, self.player_vehicle)
        elif commands_input.atmosphere_pressed or (
            commands_input.reset_pressed
            and self.player_vehicle.flight_state.environment is Environment.ATMOSPHERE
            and self.player_vehicle.flight_state.mode is VehicleMode.FIGHTER
        ):
            self.flight_controller.reset_atmosphere()
            self.player_vehicle.transformation.reset(VehicleMode.FIGHTER)
            self.combat.missile_fire_control.reset()
            self.player_vehicle.defenses.reset()
            self.world.countermeasures.clear()
            self.world.spawn_enemies(Environment.ATMOSPHERE, self.player_vehicle)
            reset_view = True
        if commands_input.select_radar:
            self.combat.missile_fire_control.select(SeekerType.RADAR)
        if commands_input.select_ir:
            self.combat.missile_fire_control.select(SeekerType.IR)
        if commands_input.toggle_ecm:
            self.player_vehicle.defenses.ecm_enabled = not self.player_vehicle.defenses.ecm_enabled
        if commands_input.dispense_flare:
            self.player_vehicle.defenses.dispense(
                self.player_vehicle, CountermeasureType.FLARE, self.world.countermeasures
            )
        if commands_input.dispense_chaff:
            self.player_vehicle.defenses.dispense(
                self.player_vehicle, CountermeasureType.CHAFF, self.world.countermeasures
            )
        if reset_view:
            self.camera.snap_to_vehicle()
        if commands_input.toggle_sas:
            self.flight_controller.fcc.toggle_stability()
        if commands_input.toggle_hover:
            self.flight_controller.fcc.toggle_hover(self.player_vehicle)
        if not parked and not self.flight_controller.fcc.hover_eligible(self.player_vehicle):
            self.flight_controller.fcc.clear_hover()
        if commands_input.select_target:
            self.combat.radar.select_next(self.world.targets)
        if dt > 0:
            # Match gun/target motion to flight substeps, including shot birth times.
            steps = max(1, math.ceil(dt / MAX_STEP))
            step = dt / steps
            for substep in range(steps):
                previous_position = self.player_vehicle.position.copy()
                previous_velocity = self.player_vehicle.velocity.copy()
                if not parked:
                    self.player_vehicle.animation.clear_layer()
                    self.player_vehicle.locomotion.clear_layer()
                    self.player_vehicle.transformation.update(step)
                    if not self.flight_controller.fcc.hover_eligible(self.player_vehicle):
                        self.flight_controller.fcc.clear_hover()
                    self.flight_controller.update(step, commands_input)
                    self.player_vehicle.animation.prepare(step, self.player_vehicle, commands_input)
                    self.player_vehicle.locomotion.update(step, self.player_vehicle, commands_input)
                    self.player_vehicle.animation.update()
                target_starts = {target: target.position.copy() for target in self.world.targets}
                candidate_starts = {c: c.position.copy() for c in self.world.countermeasures}
                for package in self.world.countermeasures:
                    package.update(step)
                self.world.countermeasures[:] = [c for c in self.world.countermeasures if c.alive]
                commands = [
                    enemy.fly(
                        step,
                        self.world.player_target,
                        self.combat.missiles,
                        commands_input.fire_gun,
                        self.world.countermeasures,
                    )
                    for enemy in self.world.enemies
                ]
                for target in self.world.targets:
                    if not getattr(target, "hostile", False):
                        target.update(step)
                self.combat.update(
                    step,
                    self.player_vehicle,
                    self.world.targets,
                    commands_input.fire_gun,
                    previous_position,
                    previous_velocity,
                    launch_missile=commands_input.launch_missile and substep == 0,
                    target_starts=target_starts,
                    advance_targets=False,
                    countermeasures=self.world.countermeasures,
                    candidate_starts=candidate_starts,
                )
                self.world.player_hit_flash = max(0.0, self.world.player_hit_flash - step)
                for enemy, command in zip(self.world.enemies, commands):
                    live = enemy.alive and command is not None
                    inventory = enemy.combat.missile_fire_control.inventory
                    enemy.combat.update(
                        step,
                        enemy,
                        [self.world.player_target],
                        firing=live and command.fire_gun,
                        previous_position=enemy.previous_position,
                        previous_velocity=enemy.previous_velocity,
                        launch_missile=live and command.fire_missile,
                        target_starts={self.world.player_target: previous_position},
                        advance_targets=False,
                        countermeasures=self.world.countermeasures,
                        candidate_starts=candidate_starts,
                    )
                    if enemy.combat.missile_fire_control.inventory < inventory:
                        enemy.pilot.missile_launched()
        elif not parked:
            self.flight_controller.update(dt, commands_input)
            self.player_vehicle.animation.clear_layer()
            self.player_vehicle.animation.prepare(max(0.0, dt), self.player_vehicle, commands_input)
            self.player_vehicle.locomotion.update(max(0.0, dt), self.player_vehicle, commands_input)
            self.player_vehicle.animation.update()
            self.combat.radar.update(self.player_vehicle, self.world.targets, self.combat.gun)
        self.player_vehicle.avionics.update(
            self.player_vehicle,
            self.world.player_target,
            self.world.enemies,
            [m for e in self.world.enemies for m in e.combat.missiles],
        )
        model_test = getattr(self.world, "model_test", None)
        if model_test is not None:
            model_test.update(dt)
        if not on_foot and not driving:
            self.camera.follow(self.player_vehicle, dt)
        if self.campaign is not None:
            self.campaign.update(self, dt, previous_access, previous_drive_position)
        self.helmet_information.observe(
            self.campaign, controls, self.input, self.navigation.elapsed
        )
        if self.input.survey_pressed:
            self.navigation.record()
        if self.renderer is not None:
            self.renderer.show_axes = self.show_axes
