"""Scripted energy regressions: no autopilot or velocity steering."""

import math
import unittest

import numpy as np

from engine.flight_controller import FlightController
from engine.hud import instrument_labels
from game.atmospheric_physics import PARAMETERS, calculate_forces, coefficients
from game.flight_state import FlightStatus
from game.player_vehicle import PlayerVehicle
from test_vehicle import commands


def combat_reversal(fps=120):
    vehicle = PlayerVehicle()
    controller = FlightController(vehicle)
    controller.fcc.stability_assist_enabled = False
    controller.reset_atmosphere()
    vehicle.throttle = 1.0
    minimum_speed = vehicle.speed
    minimum_altitude = vehicle.position[1]
    peak_aoa = 0.0

    def simulate(seconds, control):
        nonlocal minimum_speed, minimum_altitude, peak_aoa
        for frame in range(math.ceil(seconds * fps)):
            step = min(1 / fps, seconds - frame / fps)
            controller.update(step, control)
            if not (np.isfinite(vehicle.position).all() and np.isfinite(vehicle.velocity).all()
                    and np.isfinite(vehicle.orientation).all()
                    and np.isfinite(vehicle.aerodynamics.total_force).all()):
                raise AssertionError('Nonfinite combat maneuver')
            if vehicle.flight_state.status is not FlightStatus.FLYING:
                raise AssertionError('Combat maneuver hit ground')
            minimum_speed = min(minimum_speed, vehicle.speed)
            minimum_altitude = min(minimum_altitude, vehicle.position[1])
            peak_aoa = max(peak_aoa, abs(math.degrees(vehicle.aerodynamics.alpha)))

    # Roll for 0.6 s (~57 degrees), then hard pitch for ten seconds.
    # Existing velocity must reverse under force, not merely the aircraft nose.
    simulate(.6, commands(roll=1))
    simulate(10, commands(pitch=1))
    turn_heading = math.degrees(math.atan2(vehicle.velocity[0], -vehicle.velocity[2]))
    turn_speed = vehicle.speed
    turn_altitude = vehicle.position[1]
    # Open-loop pilot recovery: roll back toward upright, then ease nose up.
    # This is scripted input, not stabilization or steering of velocity.
    simulate(1.3, commands(roll=-1))
    simulate(2, commands(pitch=.3))
    simulate(5, commands())
    recovered_speed = vehicle.speed
    # Continue unassisted to reject a brief speed recovery followed by a sink/crash.
    simulate(5, commands())
    return dict(heading_deg=turn_heading, minimum_speed=minimum_speed,
                altitude_loss=PARAMETERS.test_altitude-minimum_altitude,
                turn_speed=turn_speed, turn_altitude=turn_altitude,
                recovered_speed=recovered_speed, sustained_speed=vehicle.speed,
                final_altitude=vehicle.position[1], final_vertical_speed=vehicle.velocity[1],
                peak_aoa=peak_aoa)


def straight_acceleration(seconds=5, fps=120):
    vehicle=PlayerVehicle(); controller=FlightController(vehicle)
    controller.reset_atmosphere();vehicle.throttle=1
    initial_acceleration=calculate_forces(vehicle).total_force / PARAMETERS.mass
    for _ in range(round(seconds*fps)):
        controller.update(1/fps,commands())
    return vehicle, initial_acceleration


class CombatEnergyTests(unittest.TestCase):
    def test_hard_180_degree_reversal_retains_recoverable_energy(self):
        result=combat_reversal()
        self.assertLess(abs(abs(result['heading_deg'])-180),20)
        self.assertGreater(result['peak_aoa'],PARAMETERS.stall_angle_deg)
        self.assertGreater(result['minimum_speed'],50)
        self.assertGreater(result['altitude_loss'],100)
        self.assertLess(result['altitude_loss'],650)
        self.assertGreater(result['turn_altitude'],350)
        self.assertGreater(result['recovered_speed'],result['turn_speed']+50)
        self.assertGreater(result['recovered_speed'],PARAMETERS.test_airspeed)
        self.assertGreater(result['final_altitude'],700)
        self.assertGreater(result['final_vertical_speed'],-10)
        self.assertGreater(result['sustained_speed'],result['recovered_speed'])

    def test_reversal_energy_is_consistent_across_frame_rates(self):
        reference=combat_reversal(120)
        for fps in (30,144):
            result=combat_reversal(fps)
            for field in ('minimum_speed','altitude_loss','turn_speed','recovered_speed'):
                self.assertAlmostEqual(result[field],reference[field],delta=.5)

    def test_full_throttle_has_strong_excess_thrust(self):
        vehicle, acceleration=straight_acceleration()
        self.assertGreater(-acceleration[2],15)
        self.assertGreater(vehicle.speed,180)
        self.assertGreater(vehicle.position[1],950)
        self.assertLess(abs(vehicle.position[1]-1000),100)
        self.assertTrue(np.isfinite(vehicle.velocity).all())

    def test_useful_high_aoa_lift_still_deteriorates_at_extremes(self):
        values=[coefficients(math.radians(angle)) for angle in (24,25,35,60,90)]
        self.assertGreater(values[1][0],1.3)
        self.assertGreater(values[2][0],1.1)
        self.assertLess(values[3][0],.4)
        self.assertLess(values[4][0],.02)
        self.assertGreater(values[2][1],values[0][1])
        self.assertGreater(values[3][1],values[2][1])
        self.assertLess(values[3][1],.5)

    def test_envelope_warning_thresholds_and_debug_values(self):
        vehicle=PlayerVehicle();FlightController(vehicle).reset_atmosphere()
        for alpha,warning in ((19,None),(20,'HIGH AOA'),(24,'STALL'),(-35,'STALL')):
            vehicle.aerodynamics.alpha=math.radians(alpha)
            labels=instrument_labels(vehicle,debug=True)
            if warning: self.assertIn(warning,labels)
            else:
                self.assertNotIn('HIGH AOA',labels);self.assertNotIn('STALL',labels)
            for prefix in ('SPD ','VSI ','AOA ','CL ','CD ','LIFT ','DRAG ','THRUST '):
                self.assertTrue(any(label.startswith(prefix) for label in labels))
