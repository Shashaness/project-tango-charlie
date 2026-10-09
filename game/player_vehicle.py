"""Player-owned 6-DOF spacecraft state, independent of the camera."""

import numpy as np

from engine.transform import Transform
from game.flight_state import FlightState, VehicleMode, Environment, FlightStatus
from game.battledroid_physics import GroundContact
from game.atmospheric_physics import AeroForces


class PlayerVehicle(Transform):
    @property
    def geographic_position(self):
        frame=getattr(self,'geographic_frame',None)
        return None if frame is None else frame.from_local(self.position)

    def __init__(self, position=(0, 0, 3)):
        super().__init__(position)
        self.flight_state = FlightState()
        self.ground_contact = GroundContact()
        self.aerodynamics = AeroForces()
        self.thrust_vector = 0.0
        self.throttle = 0.0
        self.engine_throttle = 0.0
        self.velocity = np.zeros(3, dtype=np.float64)
        self.acceleration = np.zeros(3, dtype=np.float64)
        self.pitch_rate = self.yaw_rate = self.roll_rate = 0.0

    @property
    def is_grounded(self):
        return (self.flight_state.mode in (VehicleMode.VTOL,VehicleMode.BATTLEDROID)
                and self.flight_state.environment is Environment.ATMOSPHERE
                and self.flight_state.status is FlightStatus.GROUNDED)

    @property
    def heat_signature(self):
        return .25+1.75*self.engine_throttle

    @property
    def radar_signature(self):
        return 1.0

    @property
    def ecm_enabled(self):
        return getattr(getattr(self, 'defenses', None), 'ecm_enabled', False)

    @property
    def speed(self):
        return float(np.linalg.norm(self.velocity))

    def toggle_configuration(self):
        """Immediate physical configuration change for legacy callers/fixtures.

        Normal gameplay G uses the staged TransformationController instead.

        Only available forces/geometry change. Never reset transform or momentum.
        """
        mode = self.flight_state.mode
        if mode is VehicleMode.FIGHTER:
            self.flight_state.mode = VehicleMode.VTOL
        elif mode is VehicleMode.VTOL:
            self.flight_state.mode = VehicleMode.BATTLEDROID
        elif mode is VehicleMode.BATTLEDROID:
            self.flight_state.mode = VehicleMode.FIGHTER
            if self.flight_state.status is FlightStatus.GROUNDED:
                self.flight_state.status = FlightStatus.FLYING
        else:
            raise NotImplementedError("Unknown vehicle configuration.")
