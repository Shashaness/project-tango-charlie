"""OpenGL 3.3 spatial test scene with stationary cubes and a reference grid."""


import numpy as np
from OpenGL import GL

from engine.hud import HUD
from engine.asset_paths import asset_path
from engine.model import ModelResources
from engine.gltf_loader import load_glb
from game.prototype_model import PROTOTYPE_PATH
from engine.mesh import Mesh
from engine.shader import Shader
from game.flight_state import Environment, VehicleMode
from game.missile import MotorState
from game.countermeasure import CountermeasureType


class Renderer:
    def __init__(self, world, vehicle, camera, fcc=None, combat=None):
        self.model_resources = None
        self.fighter_resources = None
        self.fighter_model = None
        self.combat = combat
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
        self.terrain_resources = None
        self.grid = None
        self.drawable = True

    def initialize(self):
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
            print('Terrain:', warning)
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
        self.combat_lines = Mesh([(0,0,0,1,1,1)] * 2, primitive=GL.GL_LINES)
        self.hud.initialize()
        transformation = getattr(self.vehicle, "transformation", None)
        self.fighter_model = transformation.model if transformation is not None else load_glb(PROTOTYPE_PATH)
        self.fighter_resources = ModelResources(self.fighter_model)
        test=getattr(self.world,"model_test",None)
        if test is not None:self.model_resources=ModelResources(test.model)

    @staticmethod
    def _create_demo_cube():
        # Four separate vertices per face allow solid, distinct face colors.
        faces = [
            # Front, back, left, right, top, bottom; cube side length is one.
            ((1.0, 0.2, 0.2), [(-.5, -.5, .5), (.5, -.5, .5), (.5, .5, .5), (-.5, .5, .5)]),
            ((0.2, 0.8, 0.3), [(.5, -.5, -.5), (-.5, -.5, -.5), (-.5, .5, -.5), (.5, .5, -.5)]),
            ((0.2, 0.4, 1.0), [(-.5, -.5, -.5), (-.5, -.5, .5), (-.5, .5, .5), (-.5, .5, -.5)]),
            ((1.0, 0.8, 0.2), [(.5, -.5, .5), (.5, -.5, -.5), (.5, .5, -.5), (.5, .5, .5)]),
            ((0.8, 0.2, 1.0), [(-.5, .5, .5), (.5, .5, .5), (.5, .5, -.5), (-.5, .5, -.5)]),
            ((0.2, 0.9, 0.9), [(-.5, -.5, -.5), (.5, -.5, -.5), (.5, -.5, .5), (-.5, -.5, .5)]),
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
        nose=(0,0,-2.0)
        tail=((-.22,-.22,1),(.22,-.22,1),(.22,.22,1),(-.22,.22,1))
        vertices=[]
        for index in range(4):
            vertices.extend([(*point,*(.9,.95,1)) for point in
                             (nose,tail[index],tail[(index+1)%4])])
        vertices.extend([(*tail[index],*(.5,.65,.9)) for index in (0,1,2,2,3,0)])
        return Mesh(vertices)

    @staticmethod
    def _create_target_mesh():
        # Bright octahedron vertices lie on the collision sphere's radius.
        top,bottom=(0,1,0),(0,-1,0)
        ring=((1,0,0),(0,0,-1),(-1,0,0),(0,0,1))
        vertices=[]
        for index in range(4):
            a,b=ring[index],ring[(index+1)%4]
            for pole,color in ((top,(1,.25,.1)),(bottom,(1,.75,.15))):
                vertices.extend([(*point,*color) for point in (pole,a,b)])
        return Mesh(vertices)

    @staticmethod
    def _create_vehicle_mesh(hostile=False):
        # Pointed -Z nose, wide wings, raised top and distinct lower keel.
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
        return Mesh([(*point, *((1., .15+.1*index, .35) if hostile else color))
                     for index, (points, color) in enumerate(faces) for point in points])

    @staticmethod
    def _create_battledroid_mesh():
        # COM-local humanoid boxes; upright soles at -3.2, head top at +2.2 m.
        vertices=[]
        faces=((0,1,3,2),(4,6,7,5),(0,4,5,1),(2,3,7,6),(0,2,6,4),(1,5,7,3))
        pieces=(
            ((0,0,0),(1.5,2,1),(.3,.55,.95)),       # torso
            ((0,1.6,0),(1.,1.2,.8),(.9,.9,.95)),  # head
            ((-1.1,-.2,0),(.6,2.,.6),(.8,.25,.2)),
            ((1.1,-.2,0),(.6,2.,.6),(.8,.25,.2)),
            ((-.5,-1.95,0),(.65,1.9,.65),(.65,.7,.8)),
            ((.5,-1.95,0),(.65,1.9,.65),(.65,.7,.8)),
            ((-.5,-3.05,-.25),(.8,.3,1.6),(.95,.4,.15)),
            ((.5,-3.05,-.25),(.8,.3,1.6),(.95,.4,.15)),
            ((0,.1,.75),(1.2,1.6,.5),(.2,.3,.45)), # backpack
        )
        for center,size,color in pieces:
            corners=[np.asarray(center)+np.asarray(size)*np.array((x,y,z))/2
                     for x in (-1,1) for y in (-1,1) for z in (-1,1)]
            for face in faces:
                for index in (face[0],face[1],face[2],face[0],face[2],face[3]):
                    vertices.append((*corners[index],*color))
        return Mesh(vertices)

    @staticmethod
    def _create_vtol_mesh():
        # Temporary deployed engine/leg blocks; same physical transform as the body.
        vertices = []
        for side in (-1, 1):
            for center, size, color in (
                ((side * 1.1, -1.2, .8), (.65, 2.0, .8), (.65, .7, .8)),
                ((side * 1.1, -2.2, .2), (.9, .5, 1.6), (.95, .4, .15)),
            ):
                x, y, z = center
                sx, sy, sz = (value / 2 for value in size)
                corners = [(x+dx*sx, y+dy*sy, z+dz*sz)
                           for dx, dy, dz in ((-1,-1,-1),(1,-1,-1),(1,1,-1),(-1,1,-1),
                                             (-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1))]
                for face in ((0,1,2,3),(4,7,6,5),(0,4,5,1),(3,2,6,7),(0,3,7,4),(1,5,6,2)):
                    vertices.extend([(*corners[face[index]], *color) for index in (0,1,2,2,3,0)])
        return Mesh(vertices)

    @staticmethod
    def _create_vehicle_axes():
        vertices = []
        for endpoint, color in (((0, 0, -4), (0.2, 0.4, 1)),
                                ((4, 0, 0), (1, 0.2, 0.2)),
                                ((0, 4, 0), (0.2, 1, 0.2))):
            vertices.extend([ (0, 0, 0, *color), (*endpoint, *color)])
        return Mesh(vertices, primitive=GL.GL_LINES)

    def resize(self, window, width, height):
        GL.glViewport(0, 0, width, height)
        self.hud.resize(width, height)
        # Minimized windows may have a zero-sized framebuffer.
        self.drawable = width > 0 and height > 0
        if self.drawable:
            self.camera.aspect = width / height

    @staticmethod
    def _create_reference_grid():
        # XZ ground plane at Y=0; red +X, blue -Z, green +Y.
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
        if not self.drawable:
            return
        atmosphere = self.vehicle.flight_state.environment is Environment.ATMOSPHERE
        GL.glClearColor(*((.48,.72,.91,1.) if atmosphere else (.03,.07,.12,1.)))
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
        self.hud.terrain_debug = None
        if atmosphere:
            self.environment_mesh.draw()
            if self.terrain_resources is not None:
                self.terrain_resources.update(self.camera.position, self.camera.projection_matrix() @ self.camera.view_matrix())
                self.terrain_resources.draw()
                self.hud.terrain_debug = self.terrain_resources.stats
            else:
                self.hud.terrain_debug = dict(lod='NONE',patches=0,triangles=0,agl=float(self.camera.position[1]),tiles=0)
        else:
            self.grid.draw()
        if self.camera.mode == "CHASE":
            self.shader.set_matrix("model", self.vehicle.model_matrix())
            if getattr(self.vehicle, "transformation", None) is not None and self.fighter_resources is not None:
                self.fighter_resources.draw(self.fighter_model, self.shader, self.vehicle.model_matrix())
            elif self.vehicle.flight_state.mode is VehicleMode.BATTLEDROID:
                self.battledroid_mesh.draw()
            elif self.vehicle.flight_state.mode is VehicleMode.FIGHTER and self.fighter_resources is not None:
                self.fighter_resources.draw(self.fighter_model, self.shader, self.vehicle.model_matrix())
            else:
                self.vehicle_mesh.draw()
                if self.vehicle.flight_state.mode is VehicleMode.VTOL:
                    self.vtol_mesh.draw()
            if self.show_axes:
                if (self.vehicle.flight_state.environment is Environment.ATMOSPHERE or
                        self.vehicle.flight_state.mode is VehicleMode.VTOL):
                    self._draw_force_vectors()
                else:
                    self.vehicle_axes.draw()
        test=getattr(self.world,'model_test',None)
        if test is not None and self.model_resources is not None:
            self.model_resources.draw(test.model,self.shader,self.vehicle.model_matrix()@test.root)
        self._draw_combat_scene()
        self.hud.render(self.vehicle, self.camera, self.shader, self.fcc, self.show_axes, self.combat)

    def _draw_combat_scene(self):
        for target in self.world.targets:
            if target.alive:
                if getattr(target, 'hostile', False):
                    self.shader.set_matrix('model', target.model_matrix())
                    self.enemy_mesh.draw()
                else:
                    model=np.eye(4,dtype=np.float32)
                    model[:3,:3]*=target.radius
                    model[:3,3]=target.position
                    self.shader.set_matrix('model',model)
                    self.target_mesh.draw()
        if self.combat is None:
            return
        vertices=[]

        def line(start,end,color):
            vertices.extend([(*start,*color),(*end,*color)])

        systems = [self.combat] + [enemy.combat for enemy in self.world.enemies]
        for missile in (m for system in systems for m in system.missiles):
            self.shader.set_matrix('model',missile.model_matrix())
            self.missile_mesh.draw()
            if missile.motor_state is MotorState.BURNING:
                tail=missile.position-missile.forward
                line(tail,tail-missile.forward*8,(1,.5,.1))
            if self.show_axes:
                line(missile.position,missile.position+missile.velocity*.2,(1,.8,.2))
                line(missile.position,missile.position+missile.guidance_acceleration*.1,(.8,.3,1))
                if missile.original_target.alive:line(missile.position,missile.original_target.position,(.2,.8,1))
                tracked=missile.current_seeker_target
                if tracked.alive:line(missile.position,tracked.position,(1,.2,1))
                length=100.;radius=length*np.tan(np.radians(missile.seeker.half_cone_deg))
                center=missile.position+missile.forward*length
                for i in range(12):
                    angle=i*np.pi/6;next_angle=(i+1)*np.pi/6
                    a=center+radius*(missile.right*np.cos(angle)+missile.up*np.sin(angle))
                    b=center+radius*(missile.right*np.cos(next_angle)+missile.up*np.sin(next_angle))
                    line(a,b,(.4,.4,.7))
                    if i%3==0:line(missile.position,a,(.4,.4,.7))
                for candidate,score in getattr(missile.seeker,'candidate_scores',{}).items():
                    if getattr(candidate,'kind',None) is not None and candidate.alive:
                        line(missile.position,candidate.position,(.9,.6,.2) if score>0 else (.3,.3,.3))
        for projectile in (p for system in systems for p in system.projectiles):
            speed=np.linalg.norm(projectile.velocity)
            if speed>1e-8:
                tail=projectile.position-projectile.velocity/speed*10
                line(tail,projectile.position,(1,1,.6))
        for package in self.world.countermeasures:
            if not package.alive:continue
            if package.kind is CountermeasureType.FLARE:
                for axis in np.eye(3):line(package.position-axis,package.position+axis,(1,.8,.2))
            else:
                radius=package.dispersion_radius
                for i in range(12):
                    a=i*np.pi/6;b=(i+1)*np.pi/6
                    line(package.position+radius*np.array((np.cos(a),0,np.sin(a))),
                         package.position+radius*np.array((np.cos(b),0,np.sin(b))),(.6,.8,.9))
                line(package.position-np.array((0,radius,0)),package.position+np.array((0,radius,0)),(.6,.8,.9))
        radar=self.combat.radar
        if self.show_axes and radar.current_target is not None and radar.current_target.alive:
            target=radar.current_target
            if getattr(target, 'hostile', False):
                line(target.position,target.position+target.forward*150,(1,.2,.2))
                line(target.position,target.position+target.pilot.desired_direction*200,(.2,1,.2))
                line(target.position,target.position+target.pilot.lead_direction*250,(1,.2,1))
            line(target.position,target.position+target.velocity*2,(.2,.8,1))
            if radar.track is not None:
                line(target.position,target.position+radar.track.relative_velocity*2,(.7,.4,1))
            if radar.solution is not None:
                point=radar.solution.intercept_position
                muzzle=self.combat.gun.muzzle_position(self.vehicle)
                line(muzzle,point,(1,.3,1))
                for axis in np.eye(3):line(point-axis*10,point+axis*10,(1,.3,1))
        if vertices:
            data=np.ascontiguousarray(vertices,dtype=np.float32)
            self.shader.set_matrix('model',np.eye(4,dtype=np.float32))
            GL.glBindBuffer(GL.GL_ARRAY_BUFFER,self.combat_lines.vbo)
            GL.glBufferData(GL.GL_ARRAY_BUFFER,data.nbytes,data,GL.GL_STREAM_DRAW)
            GL.glBindBuffer(GL.GL_ARRAY_BUFFER,0)
            self.combat_lines.count=len(vertices)
            self.combat_lines.draw()

    def _draw_force_vectors(self):
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
            vertices.extend([(*self.vehicle.position, *color),
                             (*(self.vehicle.position + offset), *color)])
        data = np.ascontiguousarray(vertices, dtype=np.float32)
        self.shader.set_matrix("model", np.eye(4, dtype=np.float32))
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self.force_vectors.vbo)
        GL.glBufferData(GL.GL_ARRAY_BUFFER, data.nbytes, data, GL.GL_STREAM_DRAW)
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, 0)
        self.force_vectors.draw()

    def close(self):
        """Release GPU resources while the window's context is still current."""
        if self.terrain_resources is not None:
            self.terrain_resources.close(); self.terrain_resources = None
        if self.model_resources is not None:
            self.model_resources.close();self.model_resources=None
        if self.fighter_resources is not None:
            self.fighter_resources.close()
            self.fighter_resources = None
        self.hud.close()
        for resource in ("environment_mesh", "battledroid_mesh", "enemy_mesh", "missile_mesh", "combat_lines", "target_mesh", "force_vectors", "vehicle_axes", "vtol_mesh", "vehicle_mesh"):
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
