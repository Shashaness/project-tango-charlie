"""Spatial test scenery and independent constant-velocity combat targets."""

from math import atan2

from game.ai_config import AI, AI_ENEMY_COUNT
from game.enemy_aircraft import EnemyAircraft
from game.flight_state import Environment
from game.target import Target
from game.world_loader import SceneRegistry, default_scene
from game.world_manifest import WorldDefinition, load_manifest


class World:
    """Own scenery, terrain support, targets, enemies, and shared countermeasure state."""

    def __init__(
        self, enemy_count=AI_ENEMY_COUNT, ai_parameters=AI, terrain_config=None, world_manifest=None
    ):
        """Generate scenery/terrain, create targets, and spawn the configured enemies.

        Args:
            enemy_count: Number of hostile aircraft to spawn.
            ai_parameters: Immutable AI tuning parameters.
            terrain_config: Optional terrain source/LOD configuration.
        """
        from engine.jeep_asset import jeep_access_points
        from game.ground_environment import GroundEnvironment
        from game.ground_vehicle import GroundVehicle
        from game.terrain import Terrain
        from game.world_environment import CITY_CENTER, WorldEnvironment

        if world_manifest is None:
            self.scene = default_scene()
        else:
            definition = (
                world_manifest
                if isinstance(world_manifest, WorldDefinition)
                else load_manifest(world_manifest)
            )
            self.scene = SceneRegistry(definition)
        self.aircraft_spawn = self.scene.of_kind("aircraft")[0]
        self.pilot_spawn = self.scene.of_kind("character")[0]
        self.environment = WorldEnvironment(scene=self.scene)
        self.terrain = Terrain(
            terrain_config or self.scene.definition.terrain, self.environment.protected_footprints
        )
        self.ground = GroundEnvironment(self.environment, self.terrain)
        self.environment.attach_access_roads(self.ground)
        self.live_entities = {}
        self.ground_vehicles = []
        self.ground_vehicle_assets = {}
        for placement in self.scene.of_kind("ground_vehicle"):
            path = placement.definition.asset.path
            fx, _, fz = placement.forward
            vehicle = GroundVehicle(
                placement.identity,
                placement.position,
                atan2(fx, fz),
                anchors=jeep_access_points(path),
            )
            vehicle.place(self.ground)
            vehicle.parking_brake = placement.definition.initial_state["parking_brake"]
            self.ground_vehicles.append(vehicle)
            self.ground_vehicle_assets[placement.identity] = path
            self.live_entities[placement.identity] = vehicle
        self.ground.mobile_obstacles = self.ground_vehicles

        self.geographic_anchors = dict(
            city=self.terrain.frame.from_local((CITY_CENTER[0], 0, CITY_CENTER[1])),
            airport=self.terrain.frame.from_local(self.environment.runway.center),
        )
        self.targets = [
            Target(1, (0, 0, -1000)),
            Target(2, (-500, 100, -1500), (50, 0, 0)),
            Target(3, (300, -200, -2000), (0, 30, 0)),
            Target(4, (0, 200, -2500), (0, 0, -75)),
            Target(5, (-800, 300, -3000), (60, 0, 20)),
            Target(6, (700, 500, -3500), (-50, 0, 30)),
            Target(7, (-1200, -100, -4000), (80, 20, 0)),
            Target(8, (1000, 400, -4500), (-70, -10, 20)),
            Target(9, (-1500, 600, -5000), (100, 0, 40)),
            Target(10, (1400, -300, -5500), (-90, 30, 10)),
        ]
        self.cube_positions = (
            (0, 0, -10),
            (10, 0, -25),
            (-10, 5, -40),
            (0, -10, -60),
            (20, 15, -100),
        )

        self.enemy_count = enemy_count
        self.ai_parameters = ai_parameters
        self.countermeasures = []
        self.enemies = []
        self.player_target = None
        self.player_hits = 0
        self.player_hit_flash = 0.0
        self.player_hit_weapon = "GUN"
        self.spawn_enemies(Environment.SPACE)

    def spawn_enemies(self, environment, player=None):
        """Replace hostile aircraft using the selected environment and optional player origin.

        Args:
            environment: Physics environment or shared scenery data, as required by the
                component.
            player: Player aircraft or live kinematics proxy.
        """
        for enemy in self.enemies:
            enemy.alive = False
        self.targets = [target for target in self.targets if not getattr(target, "hostile", False)]
        origin = (0, 1000, 0) if environment is Environment.ATMOSPHERE else (0, 0, 3)
        if player is not None:
            origin = player.position
        self.enemies = [
            EnemyAircraft(
                11 + i,
                environment,
                (
                    origin[0] + (-1 if i % 2 else 1) * (350 + 180 * i),
                    origin[1] + 100 + 60 * i,
                    origin[2] + 650 + 400 * i,
                ),
                self.ai_parameters,
                i,
            )
            for i in range(self.enemy_count)
        ]
        self.targets.extend(self.enemies)
        for enemy in self.enemies:
            enemy.combat.on_hit = self.player_hit

    def player_hit(self, target, weapon):
        """Record hit count, weapon, and flash telemetry without ending the game.

        Args:
            target: Target entity, desired value, or configuration.
            weapon: Weapon name used in hit accounting.
        """
        self.player_hits += 1
        self.player_hit_weapon = weapon
        self.player_hit_flash = 1.0

    @property
    def hostile_count(self):
        return sum(enemy.alive for enemy in self.enemies)

    @property
    def missile_warning(self):
        return any(
            m.alive and m.original_target is self.player_target
            for enemy in self.enemies
            for m in enemy.combat.missiles
        )
