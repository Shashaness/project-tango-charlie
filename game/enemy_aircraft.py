"""Physical hostile fighter: conservative reuse of the complete player physics."""
from dataclasses import replace
from engine.flight_controller import FlightController
from game.player_vehicle import PlayerVehicle
from game.ai_pilot import AIPilot, AIState
from game.ai_config import AI
from game.flight_state import Environment, FlightStatus
from game.weapons import CombatSystem
from game.gun import Gun, GUN
from game.countermeasure import CountermeasureDispenser
from game.defensive_avionics import DefensiveAvionics
from game.missile_seeker import SeekerType

class PlayerTarget:
    """Live read-only kinematics proxy. Hits are reported without ending the game."""
    target_id = 0
    radius = 3.0
    alive = True
    def __init__(self, vehicle): self.vehicle = vehicle
    @property
    def position(self): return self.vehicle.position
    @property
    def velocity(self): return self.vehicle.velocity
    @property
    def forward(self): return self.vehicle.forward
    @property
    def heat_signature(self): return self.vehicle.heat_signature
    @property
    def radar_signature(self): return self.vehicle.radar_signature
    @property
    def ecm_enabled(self): return self.vehicle.ecm_enabled
    def update(self, dt): pass  # player physics already ran

class EnemyAircraft(PlayerVehicle):
    hostile = True
    radius = 5.0
    def __init__(self, target_id, environment, position, parameters=AI, seed=0):
        super().__init__()
        self.target_id = target_id
        self.alive = True
        self.controller = FlightController(self)
        self.pilot = AIPilot(parameters, seed)
        self.defenses = CountermeasureDispenser(10,10)
        self.defenses.ecm_enabled = parameters.ecm_enabled and seed%2==1
        self.avionics = DefensiveAvionics()
        self.combat = CombatSystem()
        self.combat.gun = Gun(replace(GUN, firing_tolerance_deg=parameters.firing_tolerance_deg))
        self.combat.missile_fire_control.inventory = parameters.missile_inventory
        self.combat.missile_fire_control.select(SeekerType.IR if seed%2 else SeekerType.RADAR)
        # Spawn initialization only. All subsequent maneuvering goes through controls.
        if environment is Environment.ATMOSPHERE:
            self.controller.reset_atmosphere()
        else:
            self.velocity[:] = (0,0,-20)
            self.throttle = .5
        self.position[:] = position
        self.previous_position = self.position.copy()
        self.previous_velocity = self.velocity.copy()

    def update(self, dt): pass  # CombatSystem must not integrate an aircraft a second time

    def fly(self, dt, player, incoming=(), player_firing=False, countermeasures=None):
        if not self.alive:
            self.pilot.state = AIState.DEAD
            return None
        self.avionics.update(self,self,missiles=incoming)
        if countermeasures is not None:
            self.defenses.defend(dt,self,self.avionics,countermeasures)
        control=self.combat.missile_fire_control
        if control.selected_inventory == 0:
            other=SeekerType.IR if control.selected_type is SeekerType.RADAR else SeekerType.RADAR
            if control.inventories[other]>0: control.select(other)
        self.combat.radar.update(self, [player], self.combat.gun)
        command = self.pilot.update(dt, self, player, self.combat.radar,
                                    self.combat.missile_fire_control, incoming, player_firing)
        self.previous_position = self.position.copy()
        self.previous_velocity = self.velocity.copy()
        self.controller.update(dt, command)
        if self.flight_state.status is not FlightStatus.FLYING:
            self.alive = False
            self.pilot.state = AIState.DEAD
        return command
