"""OpenGL 3.3 spatial test scene with stationary cubes and a reference grid."""

import numpy as np
from OpenGL import GL

from engine.asset_paths import asset_path
from engine.gltf_loader import load_glb
from engine.hud import HUD
from engine.jeep_asset import JeepVisual
from engine.mesh import Mesh, upload_base_color
from engine.model import ModelResources
from engine.pilot_asset import PilotAsset
from engine.shader import Shader
from game.countermeasure import CountermeasureType
from game.flight_state import Environment, VehicleMode
from game.missile import MotorState
from game.player_control import UP, ControlMode
from game.prototype_model import PROTOTYPE_PATH


class Renderer:
    """Own scene GPU resources and draw vehicle, terrain, combat, and HUD observations."""

    def __init__(
        self,
        world,
        vehicle,
        camera,
        fcc=None,
        combat=None,
        *,
        campaign_hud=None,
        object_scene=None,
        navigation=None,
        information=None,
    ):
        """Retain scene observers and initialize empty GPU resource handles.

        Args:
            world: Shared world scenery, terrain, and target collections.
            vehicle: Vehicle whose pose, motion, and configuration provide simulation state.
            camera: Observational camera supplying view/projection matrices.
            fcc: Flight control computer supplying bounded commands and assist state.
            combat: Optional combat state supplying targets, tracks, and weapon telemetry.
        """
        self.model_resources = None
        self.fighter_resources = None
        self.fighter_model = None
        self.combat = combat
        self.campaign_hud = campaign_hud
        self.navigation = navigation
        self.information = information
        self.object_scene = object_scene
        self.object_mesh = None
        self.object_models = {}
        self.object_revision = -1
        self.missile_mesh = None
        self.target_mesh = None
        self.enemy_mesh = None
        self.combat_lines = None
        self.fcc = fcc
        self.hud = HUD()
        self.shader = None
        self.mesh = None
        self.camera = camera
        self.world = world
        self.vehicle = vehicle
        self.battledroid_mesh = None
        self.vtol_mesh = None
        self.vehicle_mesh = None
        self.force_vectors = None
        self.vehicle_axes = None
        self.show_axes = False
        self.environment_mesh = None
        self.pilot_meshes = []
        self.character_shader = None
        self.jeep_models = {}
        self.pilot_asset = None
        self._observed_pilot = None
        self._pilot_revision = -1
        self._pilot_time = 0.0
        self.terrain_resources = None
        self.grid = None
        self.drawable = True

    def initialize(self):
        """Compile shaders and upload scene meshes with a current OpenGL context."""
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glClearColor(0.03, 0.07, 0.12, 1.0)
        print("OpenGL information")
        for label, name in (
            ("Vendor", GL.GL_VENDOR),
            ("Renderer", GL.GL_RENDERER),
            ("Version", GL.GL_VERSION),
        ):
            value = GL.glGetString(name)
            description = value.decode("utf-8", errors="replace") if value else "Unknown"
            print(f"{label}: {description}")

        shader_dir = asset_path("shaders")
        self.shader = Shader(shader_dir / "basic.vert", shader_dir / "basic.frag")
        self.mesh = self._create_demo_cube()
        self.grid = self._create_reference_grid()
        self.environment_mesh = Mesh(self.world.environment.vertices)
        for warning in self.world.terrain.dataset.warnings:
            print("Terrain:", warning)
        if self.world.terrain is not None:
            from engine.terrain_renderer import TerrainResources

            self.terrain_resources = TerrainResources(self.world.terrain)
        self.vehicle_mesh = self._create_vehicle_mesh()
        self.enemy_mesh = self._create_vehicle_mesh(hostile=True)
        self.vtol_mesh = self._create_vtol_mesh()
        self.battledroid_mesh = self._create_battledroid_mesh()
        self.vehicle_axes = self._create_vehicle_axes()
        self.force_vectors = Mesh([(0, 0, 0, 1, 1, 1)] * 8, primitive=GL.GL_LINES)
        self.target_mesh = self._create_target_mesh()
        self.missile_mesh = self._create_missile_mesh()
        self.combat_lines = Mesh([(0, 0, 0, 1, 1, 1)] * 2, primitive=GL.GL_LINES)
        self.hud.initialize()
        transformation = getattr(self.vehicle, "transformation", None)
        self.fighter_model = (
            transformation.model if transformation is not None else load_glb(PROTOTYPE_PATH)
        )
        self.fighter_resources = ModelResources(self.fighter_model)
        for path in dict.fromkeys(getattr(self.world, "ground_vehicle_assets", {}).values()):
            visual = JeepVisual(path)
            self.jeep_models[path] = visual, ModelResources(visual.model)
        test = getattr(self.world, "model_test", None)
        if test is not None:
            self.model_resources = ModelResources(test.model)

    @staticmethod
    def _create_demo_cube():
        # Four separate vertices per face allow solid, distinct face colors.
        """Upload a unit demonstration cube with independent face colors.

        Returns:
            Uploaded Mesh; requires a current OpenGL context.
        """
        faces = [
            # Front, back, left, right, top, bottom; cube side length is one.
            (
                (1.0, 0.2, 0.2),
                [(-0.5, -0.5, 0.5), (0.5, -0.5, 0.5), (0.5, 0.5, 0.5), (-0.5, 0.5, 0.5)],
            ),
            (
                (0.2, 0.8, 0.3),
                [(0.5, -0.5, -0.5), (-0.5, -0.5, -0.5), (-0.5, 0.5, -0.5), (0.5, 0.5, -0.5)],
            ),
            (
                (0.2, 0.4, 1.0),
                [(-0.5, -0.5, -0.5), (-0.5, -0.5, 0.5), (-0.5, 0.5, 0.5), (-0.5, 0.5, -0.5)],
            ),
            (
                (1.0, 0.8, 0.2),
                [(0.5, -0.5, 0.5), (0.5, -0.5, -0.5), (0.5, 0.5, -0.5), (0.5, 0.5, 0.5)],
            ),
            (
                (0.8, 0.2, 1.0),
                [(-0.5, 0.5, 0.5), (0.5, 0.5, 0.5), (0.5, 0.5, -0.5), (-0.5, 0.5, -0.5)],
            ),
            (
                (0.2, 0.9, 0.9),
                [(-0.5, -0.5, -0.5), (0.5, -0.5, -0.5), (0.5, -0.5, 0.5), (-0.5, -0.5, 0.5)],
            ),
        ]
        vertices = []
        indices = []
        for color, corners in faces:
            start = len(vertices)
            vertices.extend([(*position, *color) for position in corners])
            indices.extend(start + index for index in (0, 1, 2, 2, 3, 0))
        return Mesh(vertices, indices)

    @staticmethod
    def _create_missile_mesh():
        """Upload the simple missile display geometry.

        Returns:
            Uploaded Mesh; requires a current OpenGL context.
        """
        nose = (0, 0, -2.0)
        tail = ((-0.22, -0.22, 1), (0.22, -0.22, 1), (0.22, 0.22, 1), (-0.22, 0.22, 1))
        vertices = []
        for index in range(4):
            vertices.extend(
                [(*point, *(0.9, 0.95, 1)) for point in (nose, tail[index], tail[(index + 1) % 4])]
            )
        vertices.extend([(*tail[index], *(0.5, 0.65, 0.9)) for index in (0, 1, 2, 2, 3, 0)])
        return Mesh(vertices)

    @staticmethod
    def _create_target_mesh():
        # Bright octahedron vertices lie on the collision sphere's radius.
        """Upload octahedral target-marker geometry.

        Returns:
            Uploaded Mesh; requires a current OpenGL context.
        """
        top, bottom = (0, 1, 0), (0, -1, 0)
        ring = ((1, 0, 0), (0, 0, -1), (-1, 0, 0), (0, 0, 1))
        vertices = []
        for index in range(4):
            a, b = ring[index], ring[(index + 1) % 4]
            for pole, color in ((top, (1, 0.25, 0.1)), (bottom, (1, 0.75, 0.15))):
                vertices.extend([(*point, *color) for point in (pole, a, b)])
        return Mesh(vertices)

    @staticmethod
    def _create_vehicle_mesh(hostile=False):
        # Pointed -Z nose, wide wings, raised top and distinct lower keel.
        """Upload the primitive fighter geometry with friendly or hostile colors.

        Args:
            hostile: Whether to use hostile aircraft display colors.

        Returns:
            Uploaded Mesh; requires a current OpenGL context.
        """
        nose = (0, 0, -2.5)
        left, right = (-2, 0, 1.5), (2, 0, 1.5)
        top, bottom = (0, 0.8, 1), (0, -0.5, 1)
        faces = (
            ((nose, left, top), (0.2, 0.5, 1.0)),
            ((nose, top, right), (1.0, 0.65, 0.15)),
            ((nose, bottom, left), (0.12, 0.18, 0.35)),
            ((nose, right, bottom), (0.4, 0.18, 0.08)),
            ((left, bottom, top), (0.2, 0.4, 0.8)),
            ((right, top, bottom), (0.8, 0.4, 0.1)),
        )
        return Mesh(
            [
                (*point, *((1.0, 0.15 + 0.1 * index, 0.35) if hostile else color))
                for index, (points, color) in enumerate(faces)
                for point in points
            ]
        )

    @staticmethod
    def _create_battledroid_mesh():
        # COM-local humanoid boxes; upright soles at -3.2, head top at +2.2 m.
        """Upload the primitive battledroid display geometry.

        Returns:
            Uploaded Mesh; requires a current OpenGL context.
        """
        vertices = []
        faces = ((0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3))
        pieces = (
            ((0, 0, 0), (1.5, 2, 1), (0.3, 0.55, 0.95)),  # torso
            ((0, 1.6, 0), (1.0, 1.2, 0.8), (0.9, 0.9, 0.95)),  # head
            ((-1.1, -0.2, 0), (0.6, 2.0, 0.6), (0.8, 0.25, 0.2)),
            ((1.1, -0.2, 0), (0.6, 2.0, 0.6), (0.8, 0.25, 0.2)),
            ((-0.5, -1.95, 0), (0.65, 1.9, 0.65), (0.65, 0.7, 0.8)),
            ((0.5, -1.95, 0), (0.65, 1.9, 0.65), (0.65, 0.7, 0.8)),
            ((-0.5, -3.05, -0.25), (0.8, 0.3, 1.6), (0.95, 0.4, 0.15)),
            ((0.5, -3.05, -0.25), (0.8, 0.3, 1.6), (0.95, 0.4, 0.15)),
            ((0, 0.1, 0.75), (1.2, 1.6, 0.5), (0.2, 0.3, 0.45)),  # backpack
        )
        for center, size, color in pieces:
            corners = [
                np.asarray(center) + np.asarray(size) * np.array((x, y, z)) / 2
                for x in (-1, 1)
                for y in (-1, 1)
                for z in (-1, 1)
            ]
            for face in faces:
                for index in (face[0], face[1], face[2], face[0], face[2], face[3]):
                    vertices.append((*corners[index], *color))
        return Mesh(vertices)

    @staticmethod
    def _create_vtol_mesh():
        # Temporary deployed engine/leg blocks; same physical transform as the body.
        """Upload the primitive VTOL display geometry.

        Returns:
            Uploaded Mesh; requires a current OpenGL context.
        """
        vertices = []
        for side in (-1, 1):
            for center, size, color in (
                ((side * 1.1, -1.2, 0.8), (0.65, 2.0, 0.8), (0.65, 0.7, 0.8)),
                ((side * 1.1, -2.2, 0.2), (0.9, 0.5, 1.6), (0.95, 0.4, 0.15)),
            ):
                x, y, z = center
                sx, sy, sz = (value / 2 for value in size)
                corners = [
                    (x + dx * sx, y + dy * sy, z + dz * sz)
                    for dx, dy, dz in (
                        (-1, -1, -1),
                        (1, -1, -1),
                        (1, 1, -1),
                        (-1, 1, -1),
                        (-1, -1, 1),
                        (1, -1, 1),
                        (1, 1, 1),
                        (-1, 1, 1),
                    )
                ]
                for face in (
                    (0, 1, 2, 3),
                    (4, 7, 6, 5),
                    (0, 4, 5, 1),
                    (3, 2, 6, 7),
                    (0, 3, 7, 4),
                    (1, 5, 6, 2),
                ):
                    vertices.extend(
                        [(*corners[face[index]], *color) for index in (0, 1, 2, 2, 3, 0)]
                    )
        return Mesh(vertices)

    @staticmethod
    def _create_vehicle_axes():
        """Upload colored local-axis diagnostic lines.

        Returns:
            Uploaded Mesh; requires a current OpenGL context.
        """
        vertices = []
        for endpoint, color in (
            ((0, 0, -4), (0.2, 0.4, 1)),
            ((4, 0, 0), (1, 0.2, 0.2)),
            ((0, 4, 0), (0.2, 1, 0.2)),
        ):
            vertices.extend([(0, 0, 0, *color), (*endpoint, *color)])
        return Mesh(vertices, primitive=GL.GL_LINES)

    def resize(self, window, width, height):
        """Update viewport dimensions.

        Args:
            window: GLFW window handle.
            width: Viewport width in pixels.
            height: Viewport height in pixels.
        """
        GL.glViewport(0, 0, width, height)
        self.hud.resize(width, height)
        # Minimized windows may have a zero-sized framebuffer.
        self.drawable = width > 0 and height > 0
        if self.drawable:
            self.camera.aspect = width / height

    @staticmethod
    def _create_reference_grid():
        # XZ ground plane at Y=0; red +X, blue -Z, green +Y.
        """Upload the static world-reference line grid.

        Returns:
            Uploaded Mesh; requires a current OpenGL context.
        """
        vertices = []

        def line(start, end, color):
            vertices.extend([(*start, *color), (*end, *color)])

        for coordinate in range(-100, 101, 5):
            color = (0.18, 0.24, 0.30)
            line((coordinate, 0, -100), (coordinate, 0, 100), color)
            line((-100, 0, coordinate), (100, 0, coordinate), color)
        # Sparse large ground reference remains visible from the 1000 m test start.
        for coordinate in range(-20000, 20001, 1000):
            line((coordinate, 0, -20000), (coordinate, 0, 20000), (0.12, 0.18, 0.23))
            line((-20000, 0, coordinate), (20000, 0, coordinate), (0.12, 0.18, 0.23))
        line((0, 0, 0), (10, 0, 0), (1, 0.2, 0.2))
        line((0, 0, 0), (0, 10, 0), (0.2, 1, 0.2))
        line((0, 0, 0), (0, 0, -10), (0.2, 0.4, 1))
        return Mesh(vertices, primitive=GL.GL_LINES)

    def render(self):
        """Draw a frame from world and camera observations."""
        if not self.drawable:
            return
        atmosphere = self.vehicle.flight_state.environment is Environment.ATMOSPHERE
        GL.glClearColor(*((0.48, 0.72, 0.91, 1.0) if atmosphere else (0.03, 0.07, 0.12, 1.0)))
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        self.shader.use()
        self.shader.set_matrix("view", self.camera.view_matrix())
        self.shader.set_matrix("projection", self.camera.projection_matrix())
        for position in self.world.cube_positions:
            model = np.eye(4, dtype=np.float32)
            model[:3, 3] = position
            self.shader.set_matrix("model", model)
            self.mesh.draw()
        self.shader.set_matrix("model", np.eye(4, dtype=np.float32))
        control = getattr(self.world, "player_control", None)
        on_foot = control is not None and control.mode is ControlMode.ON_FOOT
        driving = control is not None and control.mode is ControlMode.GROUND_VEHICLE
        terrain_observer = (
            control.pilot if on_foot else control.active.vehicle if driving else self.vehicle
        )
        self.hud.terrain_debug = None
        geographic = self.vehicle.geographic_position
        self.hud.geographic_mapped = (
            geographic is not None
            and self.world.terrain.dataset.has_coverage(geographic.latitude, geographic.longitude)
        )
        if atmosphere:
            self.environment_mesh.draw()
            if self.terrain_resources is not None:
                self.terrain_resources.update(
                    self.camera.position,
                    self.camera.projection_matrix() @ self.camera.view_matrix(),
                    terrain_observer.position,
                    terrain_observer.velocity,
                )
                self.terrain_resources.draw(self.shader, self.camera)
                self.hud.terrain_debug = self.terrain_resources.stats
            else:
                self.hud.terrain_debug = dict(
                    lod="NONE", patches=0, triangles=0, agl=float(self.camera.position[1]), tiles=0
                )
        else:
            self.grid.draw()
        if atmosphere:
            self._draw_ground_vehicles()
            self._draw_interactive_objects()
        if self.camera.mode == "CHASE" or on_foot or driving:
            self.shader.set_matrix("model", self.vehicle.model_matrix())
            if (
                getattr(self.vehicle, "transformation", None) is not None
                and self.fighter_resources is not None
            ):
                self.fighter_resources.draw(
                    self.fighter_model, self.shader, self.vehicle.model_matrix()
                )
            elif self.vehicle.flight_state.mode is VehicleMode.BATTLEDROID:
                self.battledroid_mesh.draw()
            elif (
                self.vehicle.flight_state.mode is VehicleMode.FIGHTER
                and self.fighter_resources is not None
            ):
                self.fighter_resources.draw(
                    self.fighter_model, self.shader, self.vehicle.model_matrix()
                )
            else:
                self.vehicle_mesh.draw()
                if self.vehicle.flight_state.mode is VehicleMode.VTOL:
                    self.vtol_mesh.draw()
            if self.show_axes and not (on_foot or driving):
                if (
                    self.vehicle.flight_state.environment is Environment.ATMOSPHERE
                    or self.vehicle.flight_state.mode is VehicleMode.VTOL
                ):
                    self._draw_force_vectors()
                else:
                    self.vehicle_axes.draw()
        test = getattr(self.world, "model_test", None)
        if test is not None and self.model_resources is not None:
            self.model_resources.draw(
                test.model, self.shader, self.vehicle.model_matrix() @ test.root
            )
        self._draw_combat_scene()
        if on_foot:
            self._draw_pilot(control.pilot, visible=not self.camera.first_person)
        elif driving:
            self._draw_driver(control.active.vehicle)
        else:
            self.hud.render(
                self.vehicle, self.camera, self.shader, self.fcc, self.show_axes, self.combat
            )
        fix, notice = self.navigation() if self.navigation else (None, "")
        inset = 0
        if fix is not None:
            inset = self.hud.render_navigation(self.shader, fix, notice)
        if (on_foot or driving) and self.information is not None:
            self.hud.render_helmet_information(self.shader, *self.information())
            return
        if control is not None:
            self.hud.render_control_prompt(self.shader, control.prompt(), row_offset=inset)
        if self.campaign_hud is not None:
            for row, line in enumerate(self.campaign_hud(), start=inset + 1):
                self.hud.render_control_prompt(self.shader, line, row_offset=row)

    def _draw_interactive_objects(self):
        scene = self.object_scene
        if scene is None:
            return
        if self.object_revision != scene.revision:
            corners = np.array(
                (
                    (0, 0, 0),
                    (1, 0, 0),
                    (1, 1, 0),
                    (0, 1, 0),
                    (0, 0, 1),
                    (1, 0, 1),
                    (1, 1, 1),
                    (0, 1, 1),
                )
            )
            faces = (
                (0, 3, 2, 1),
                (4, 5, 6, 7),
                (0, 1, 5, 4),
                (3, 7, 6, 2),
                (0, 4, 7, 3),
                (1, 2, 6, 5),
            )
            vertices = []
            for lower, upper, rgb in scene.render_rows():
                points = lower + corners * (upper - lower)
                for face, shade in zip(faces, (0.78, 0.87, 0.60, 1.0, 0.73, 0.91), strict=True):
                    color = np.asarray(rgb) * shade
                    vertices.extend((*points[face[index]], *color) for index in (0, 1, 2, 0, 2, 3))
            if vertices:
                if self.object_mesh is None:
                    self.object_mesh = Mesh(vertices)
                else:
                    self.object_mesh.update_vertices(vertices)
            elif self.object_mesh is not None:
                self.object_mesh.close()
                self.object_mesh = None
            self.object_revision = scene.revision
        if self.object_mesh is not None:
            self.shader.set_matrix("model", np.eye(4, dtype=np.float32))
            self.object_mesh.draw()
        for path, position in scene.visual_rows():
            if path not in self.object_models:
                from game.world_loader import scene_model

                model = scene_model(path)
                self.object_models[path] = model, ModelResources(model)
            model, resources = self.object_models[path]
            placement = np.eye(4)
            placement[:3, 3] = position
            resources.draw(model, self.shader, placement)

    def _draw_ground_vehicles(self):
        if not self.jeep_models:
            return
        for vehicle in getattr(self.world, "ground_vehicles", ()):
            path = self.world.ground_vehicle_assets[vehicle.identity]
            visual, resources = self.jeep_models[path]
            images = getattr(visual.model, "base_color_textures", ())
            textured_shader = self._base_color_shader() if images else None
            resources.draw(
                visual.model, self.shader, visual.pose(vehicle), textured_shader=textured_shader
            )

    def _base_color_shader(self):
        if self.character_shader is None:
            shaders = asset_path("shaders")
            self.character_shader = Shader(shaders / "basic.vert", shaders / "character.frag")
        shader = self.character_shader
        shader.use()
        shader.set_matrix("view", self.camera.view_matrix())
        shader.set_matrix("projection", self.camera.projection_matrix())
        self.shader.use()
        return shader

    def _load_pilot(self):
        if self.pilot_asset is None:
            self.pilot_asset = PilotAsset(self.world.pilot_spawn.definition.asset.path)

    def _draw_driver(self, vehicle):
        self._load_pilot()
        hips = self.pilot_asset.seat_hips
        placement = vehicle.model_matrix()
        placement[:3, 3] = vehicle.anchor("DriverSeat") + placement[:3, :3] @ (
            self.pilot_asset.seat_offset - hips
        )
        self._draw_pilot_mesh(placement, seated=True)

    def _draw_pilot(self, pilot, visible=True):
        self._load_pilot()
        if self._observed_pilot is not pilot or self._pilot_revision != pilot.pose_revision:
            self._observed_pilot = pilot
            self._pilot_revision = pilot.pose_revision
            self.pilot_asset.reset_locomotion()
            self._pilot_time = pilot.elapsed
        self.pilot_asset.update(pilot.clip, pilot.elapsed - self._pilot_time)
        self._pilot_time = pilot.elapsed
        if not visible:
            return
        placement = np.eye(4)
        placement[:3, :3] = np.column_stack((np.cross(UP, pilot.forward), UP, pilot.forward))
        placement[:3, 3] = pilot.position
        self._draw_pilot_mesh(placement)

    def _draw_pilot_mesh(self, placement, seated=False):
        parts = list(self.pilot_asset.vertices(seated=seated))
        if not self.pilot_meshes:
            for (vertices, indices), (pixels, _) in zip(
                parts, self.pilot_asset.base_colors, strict=True
            ):
                mesh = Mesh(vertices, indices)
                self.pilot_meshes.append((mesh, 0))
                if pixels is not None:
                    self.pilot_meshes[-1] = mesh, upload_base_color(pixels)
        textured_shader = (
            self._base_color_shader() if any(t for _, t in self.pilot_meshes) else None
        )
        for (values, _), (mesh, texture), (_, factor) in zip(
            parts, self.pilot_meshes, self.pilot_asset.base_colors, strict=True
        ):
            mesh.update_vertices(values)
            shader = self.shader
            if texture:
                shader = textured_shader
                shader.use()
                GL.glActiveTexture(GL.GL_TEXTURE0)
                GL.glBindTexture(GL.GL_TEXTURE_2D, texture)
                GL.glUniform3f(GL.glGetUniformLocation(shader.program, "baseFactor"), *factor)
            shader.set_matrix("model", placement)
            mesh.draw()
            if texture:
                GL.glBindTexture(GL.GL_TEXTURE_2D, 0)
                self.shader.use()

    def _draw_combat_scene(self):
        """Draw targets, projectiles, missiles, and decoys from their simulation poses."""
        for target in self.world.targets:
            if target.alive:
                if getattr(target, "hostile", False):
                    self.shader.set_matrix("model", target.model_matrix())
                    self.enemy_mesh.draw()
                else:
                    model = np.eye(4, dtype=np.float32)
                    model[:3, :3] *= target.radius
                    model[:3, 3] = target.position
                    self.shader.set_matrix("model", model)
                    self.target_mesh.draw()
        if self.combat is None:
            return
        vertices = []

        def line(start, end, color):
            vertices.extend([(*start, *color), (*end, *color)])

        if getattr(self.vehicle, "anchor_debug", False):
            from game.reference_anchors import positions

            for name, local in positions(self.vehicle).items():
                point = self.vehicle.orientation @ local + self.vehicle.position
                color = (
                    (1, 0.7, 0)
                    if name.startswith("TC_CONTACT_")
                    else (0.2, 0.7, 1)
                    if name.startswith("TC_THRUST_")
                    else (1, 0.2, 0.4)
                    if name.startswith(("TC_BODY_", "TC_HULL_"))
                    else (0.3, 1, 0.3)
                )
                for axis in np.eye(3) * 0.2:
                    line(point - axis, point + axis, color)
        systems = [self.combat] + [enemy.combat for enemy in self.world.enemies]
        for missile in (m for system in systems for m in system.missiles):
            self.shader.set_matrix("model", missile.model_matrix())
            self.missile_mesh.draw()
            if missile.motor_state is MotorState.BURNING:
                tail = missile.position - missile.forward
                line(tail, tail - missile.forward * 8, (1, 0.5, 0.1))
            if self.show_axes:
                line(missile.position, missile.position + missile.velocity * 0.2, (1, 0.8, 0.2))
                line(
                    missile.position,
                    missile.position + missile.guidance_acceleration * 0.1,
                    (0.8, 0.3, 1),
                )
                if missile.original_target.alive:
                    line(missile.position, missile.original_target.position, (0.2, 0.8, 1))
                tracked = missile.current_seeker_target
                if tracked.alive:
                    line(missile.position, tracked.position, (1, 0.2, 1))
                length = 100.0
                radius = length * np.tan(np.radians(missile.seeker.half_cone_deg))
                center = missile.position + missile.forward * length
                for i in range(12):
                    angle = i * np.pi / 6
                    next_angle = (i + 1) * np.pi / 6
                    a = center + radius * (
                        missile.right * np.cos(angle) + missile.up * np.sin(angle)
                    )
                    b = center + radius * (
                        missile.right * np.cos(next_angle) + missile.up * np.sin(next_angle)
                    )
                    line(a, b, (0.4, 0.4, 0.7))
                    if i % 3 == 0:
                        line(missile.position, a, (0.4, 0.4, 0.7))
                for candidate, score in getattr(missile.seeker, "candidate_scores", {}).items():
                    if getattr(candidate, "kind", None) is not None and candidate.alive:
                        line(
                            missile.position,
                            candidate.position,
                            (0.9, 0.6, 0.2) if score > 0 else (0.3, 0.3, 0.3),
                        )
        for projectile in (p for system in systems for p in system.projectiles):
            speed = np.linalg.norm(projectile.velocity)
            if speed > 1e-8:
                tail = projectile.position - projectile.velocity / speed * 10
                line(tail, projectile.position, (1, 1, 0.6))
        for package in self.world.countermeasures:
            if not package.alive:
                continue
            if package.kind is CountermeasureType.FLARE:
                for axis in np.eye(3):
                    line(package.position - axis, package.position + axis, (1, 0.8, 0.2))
            else:
                radius = package.dispersion_radius
                for i in range(12):
                    a = i * np.pi / 6
                    b = (i + 1) * np.pi / 6
                    line(
                        package.position + radius * np.array((np.cos(a), 0, np.sin(a))),
                        package.position + radius * np.array((np.cos(b), 0, np.sin(b))),
                        (0.6, 0.8, 0.9),
                    )
                line(
                    package.position - np.array((0, radius, 0)),
                    package.position + np.array((0, radius, 0)),
                    (0.6, 0.8, 0.9),
                )
        radar = self.combat.radar
        if self.show_axes and radar.current_target is not None and radar.current_target.alive:
            target = radar.current_target
            if getattr(target, "hostile", False):
                line(target.position, target.position + target.forward * 150, (1, 0.2, 0.2))
                line(
                    target.position,
                    target.position + target.pilot.desired_direction * 200,
                    (0.2, 1, 0.2),
                )
                line(
                    target.position,
                    target.position + target.pilot.lead_direction * 250,
                    (1, 0.2, 1),
                )
            line(target.position, target.position + target.velocity * 2, (0.2, 0.8, 1))
            if radar.track is not None:
                line(
                    target.position,
                    target.position + radar.track.relative_velocity * 2,
                    (0.7, 0.4, 1),
                )
            if radar.solution is not None:
                point = radar.solution.intercept_position
                muzzle = self.combat.gun.muzzle_position(self.vehicle)
                line(muzzle, point, (1, 0.3, 1))
                for axis in np.eye(3):
                    line(point - axis * 10, point + axis * 10, (1, 0.3, 1))
        if vertices:
            data = np.ascontiguousarray(vertices, dtype=np.float32)
            self.shader.set_matrix("model", np.eye(4, dtype=np.float32))
            GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self.combat_lines.vbo)
            GL.glBufferData(GL.GL_ARRAY_BUFFER, data.nbytes, data, GL.GL_STREAM_DRAW)
            GL.glBindBuffer(GL.GL_ARRAY_BUFFER, 0)
            self.combat_lines.count = len(vertices)
            self.combat_lines.draw()

    def _draw_force_vectors(self):
        """Build and draw diagnostic force directions without modifying physics."""
        aero = self.vehicle.aerodynamics
        vertices = []
        for vector, scale, color in (
            (aero.lift_force, 1 / 10000, (0.2, 1, 0.3)),
            (aero.drag_force, 1 / 10000, (1, 0.2, 0.2)),
            (aero.thrust_force, 1 / 10000, (0.2, 0.6, 1)),
            (self.vehicle.velocity, 1 / 20, (1, 0.8, 0.2)),
        ):
            offset = vector * scale
            length = np.linalg.norm(offset)
            if length > 12:
                offset *= 12 / length
            vertices.extend(
                [(*self.vehicle.position, *color), (*(self.vehicle.position + offset), *color)]
            )
        data = np.ascontiguousarray(vertices, dtype=np.float32)
        self.shader.set_matrix("model", np.eye(4, dtype=np.float32))
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self.force_vectors.vbo)
        GL.glBufferData(GL.GL_ARRAY_BUFFER, data.nbytes, data, GL.GL_STREAM_DRAW)
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, 0)
        self.force_vectors.draw()

    def close(self):
        """Release GPU resources while the window's context is still current."""
        if self.terrain_resources is not None:
            self.terrain_resources.close()
            self.terrain_resources = None
        if self.model_resources is not None:
            self.model_resources.close()
            self.model_resources = None
        if self.fighter_resources is not None:
            self.fighter_resources.close()
            self.fighter_resources = None
        for _, resources in self.jeep_models.values():
            resources.close()
        self.jeep_models.clear()
        self.hud.close()
        if self.object_mesh is not None:
            self.object_mesh.close()
            self.object_mesh = None
        for _, resources in self.object_models.values():
            resources.close()
        self.object_models.clear()
        for mesh, texture in self.pilot_meshes:
            mesh.close()
            if texture:
                GL.glDeleteTextures([texture])
        self.pilot_meshes.clear()
        if self.character_shader is not None:
            self.character_shader.close()
            self.character_shader = None
        for resource in (
            "environment_mesh",
            "battledroid_mesh",
            "enemy_mesh",
            "missile_mesh",
            "combat_lines",
            "target_mesh",
            "force_vectors",
            "vehicle_axes",
            "vtol_mesh",
            "vehicle_mesh",
        ):
            mesh = getattr(self, resource)
            if mesh is not None:
                mesh.close()
                setattr(self, resource, None)
        if self.grid is not None:
            self.grid.close()
            self.grid = None
        if self.mesh is not None:
            self.mesh.close()
            self.mesh = None
        if self.shader is not None:
            GL.glUseProgram(0)
            self.shader.close()
            self.shader = None
