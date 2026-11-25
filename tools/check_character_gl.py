"""Drive the M24.2 route in the game and capture the OpenGL character poses."""

import argparse
import json
import math
import tempfile
from pathlib import Path

import glfw
import numpy as np
from OpenGL import GL

from engine.game import Game
from engine.renderer import Renderer
from game.flight_state import VehicleMode
from game.player_control import UP, ControlMode
from game.world_manifest import DEFAULT_WORLD
from tools.check_transformation_gl import capture


def look(camera, target, offset):
    camera.position[:] = target + offset
    camera.forward[:] = -np.asarray(offset) / np.linalg.norm(offset)
    right = np.cross(camera.forward, UP)
    camera.right[:] = right / np.linalg.norm(right)
    camera.up[:] = np.cross(camera.right, camera.forward)


def check(window, manifest, output, legacy):
    game = Game(enemy_count=0, world_manifest=manifest)
    control = game.player_control
    aircraft = game.player_vehicle
    renderer = Renderer(game.world, aircraft, game.camera, game.flight_controller.fcc, game.combat)
    game.renderer = renderer
    steps = 0

    def draw(label, target=None, offset=None):
        if target is not None:
            look(renderer.camera, target, offset)
        renderer.render()
        if GL.glGetError() != GL.GL_NO_ERROR:
            raise RuntimeError(f"OpenGL error during {label}")
        if output:
            capture(output / f"{label}.png")
        glfw.swap_buffers(window)

    def step(dt=1 / 30):
        nonlocal steps
        game.update(dt)
        steps += 1

    def interact(expected):
        game.input.interact_pressed = True
        step()
        game.input.interact_pressed = False
        if control.mode is not expected:
            raise AssertionError(control.message)

    def walk_to(x, z, *, run=False, shot=None, shot_frame=12, radius=0.25):
        game.input.pilot_run = run
        game.input.pilot_advance = 1
        for frame in range(1500):
            pilot = control.pilot
            dx, dz = x - pilot.position[0], z - pilot.position[2]
            if math.hypot(dx, dz) <= radius:
                break
            game.pedestrian_camera.orbit_yaw = math.atan2(dx, dz)
            step()
            if frame == shot_frame and shot:
                assert pilot.clip == ("Run" if run else "Walk"), pilot.clip
                offset = (1.2, 0.72, 3.2) if run else pilot.forward * 3 + (1, 0.75, 0)
                draw(shot, pilot.position + UP * 0.91, offset)
        else:
            raise AssertionError(f"Route blocked at {pilot.position} toward {(x, z)}")
        game.input.pilot_advance = 0
        game.input.pilot_run = False
        for _ in range(12):
            step()

    def turn(side):
        # The existing controller emits Turn when requested yaw changes and translation is blocked.
        pilot, camera = control.pilot, game.pedestrian_camera
        saved = pilot.position.copy()
        try:
            near_wall = np.array((342.70, 0, 3007.0))
            pilot.position[:] = near_wall
            pilot.velocity[:] = 0
            pilot.forward[:] = (0, 0, side)
            camera.orbit_yaw = math.pi / 2
            game.input.pilot_advance = 1
            for _ in range(10):
                step(1 / 120)
            assert pilot.forward[0] > 0.1 and side * pilot.forward[2] > 0.1
            if pilot.clip != "Turn":
                raise AssertionError(f"Blocked turn selected {pilot.clip} at {pilot.position}")
            draw(
                "turn_left" if side > 0 else "turn_right",
                pilot.position + UP * 0.9,
                (-2.5, 0.65, side * 2.2),
            )
        finally:
            pilot.position[:] = saved
            pilot.velocity[:] = 0
            game.input.pilot_advance = 0
            step()

    try:
        renderer.initialize()
        renderer.resize(window, 1280, 720)
        game.camera.mode = "COCKPIT"
        game.camera.follow(aircraft, 0)
        assert control.mode is ControlMode.AIRCRAFT
        draw("01_cockpit_start")
        interact(ControlMode.ON_FOOT)
        pilot = control.pilot
        exit_position = pilot.position.copy()
        assert pilot.clip == "Idle"
        draw("02_hangar_idle", pilot.position + UP * 0.91, (-3.1, 0.7, -0.95))
        expected = "pilot.glb" if legacy else "female_pilot.glb"
        assert game.world.pilot_spawn.definition.asset.path.name == expected
        assert ("Seated" in renderer.pilot_asset.model.animations) != legacy
        walk_to(275, 2993, shot="03_hangar_walk")
        walk_to(251, 2993)
        walk_to(222, 2986, run=True, shot="04_apron_run", shot_frame=85)
        if not legacy:
            turn(1)
            turn(-1)
        walk_to(251, 2993, run=True)
        walk_to(315, 2993)
        walk_to(330, 3015)
        walk_to(337.5, 3038)
        draw("05_repair_station", pilot.position + UP * 0.95, (-4.0, 1.2, -3.2))
        walk_to(330, 3015)
        walk_to(330, 2988)
        jeep = game.world.live_entities["hangar-jeep"]
        entry = jeep.anchor("Exit.L")
        walk_to(entry[0], entry[2])
        interact(ControlMode.GROUND_VEHICLE)
        draw("06_jeep_seated", jeep.position + UP * 0.78, (-2.8, 1.7, 3.4))
        before = jeep.position.copy()
        game.input.toggle_parking_brake = True
        step()
        game.input.toggle_parking_brake = False
        game.input.pilot_advance = 1
        for _ in range(50):
            step()
        assert np.linalg.norm(jeep.position - before) > 1
        draw("07_jeep_driving", jeep.position + UP * 0.65, (-3.8, 2.25, 4.8))
        game.input.pilot_advance = 0
        game.input.fire_gun = True  # Jeep service brake
        for _ in range(65):
            step()
        game.input.fire_gun = False
        interact(ControlMode.ON_FOOT)
        assert pilot is control.pilot and pilot.clip == "Idle"
        draw("08_jeep_exit", pilot.position + UP * 0.92, (-2.4, 0.9, 2.6))
        assert renderer.pilot_asset.player.clip.name == "Idle"
        walk_to(303, 2993)
        walk_to(exit_position[0], exit_position[2])
        interact(ControlMode.AIRCRAFT)
        draw("09_reboarded")
        flaps = aircraft.airfield.flaps
        game.input.toggle_flaps = True
        step()
        game.input.toggle_flaps = False
        assert aircraft.airfield.flaps is not flaps
        game.camera.mode = "CHASE"
        for index, mode in enumerate(
            (VehicleMode.VTOL, VehicleMode.BATTLEDROID, VehicleMode.FIGHTER)
        ):
            game.input.toggle_configuration = True
            step()
            game.input.toggle_configuration = False
            for _ in range(240):
                if not aircraft.transformation.active:
                    break
                step()
            assert not aircraft.transformation.active
            assert aircraft.flight_state.mode is mode, aircraft.flight_state.mode
            draw(f"10_{index}_{mode.name.lower()}")
        print(f"{expected}: route passed in {steps} simulation updates")
    finally:
        renderer.close()
        game.world.terrain.dataset.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--legacy", action="store_true")
    args = parser.parse_args()
    if args.output:
        args.output.mkdir(parents=True, exist_ok=True)
    if not glfw.init():
        raise RuntimeError("GLFW initialization failed")
    window = None
    try:
        for key, value in (
            (glfw.CONTEXT_VERSION_MAJOR, 3),
            (glfw.CONTEXT_VERSION_MINOR, 3),
            (glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE),
            (glfw.OPENGL_FORWARD_COMPAT, glfw.TRUE),
            (glfw.VISIBLE, glfw.FALSE),
        ):
            glfw.window_hint(key, value)
        window = glfw.create_window(1280, 720, "PTC character verification", None, None)
        if not window:
            raise RuntimeError("OpenGL context creation failed")
        glfw.make_context_current(window)
        if args.legacy:
            with tempfile.TemporaryDirectory(prefix="ptc-original-pilot-") as folder:
                manifest = Path(folder) / "world.json"
                document = json.loads(DEFAULT_WORLD.read_text())
                next(e for e in document["entities"] if e["id"] == "pilot_01")["asset"] = "pilot"
                manifest.write_text(json.dumps(document))
                check(window, manifest, args.output, legacy=True)
        else:
            check(window, DEFAULT_WORLD, args.output, legacy=False)
    finally:
        if window is not None:
            glfw.destroy_window(window)
        glfw.terminate()


if __name__ == "__main__":
    main()
