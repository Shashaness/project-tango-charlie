"""Project Tango Charlie entry point and reproducible AI test presets."""

import argparse
from dataclasses import replace
from pathlib import Path

from engine.game import Game
from game.ai_config import AI, AI_ENEMY_COUNT
from game.identity import PROJECT_TITLE
from game.missile_seeker import SeekerType


def main():
    """Create the game and run its window loop, releasing resources on exit."""
    parser = argparse.ArgumentParser(description=PROJECT_TITLE)
    parser.add_argument(
        "--no-locomotion-animation",
        action="store_true",
        help="disable visual Battledroid gait; physics unchanged",
    )
    parser.add_argument(
        "--legacy-animation",
        action="store_true",
        help="use original Python poses and legacy TC167 model",
    )
    parser.add_argument("--animation-debug", action="store_true")
    parser.add_argument("--anchor-debug", action="store_true")
    parser.add_argument("--enemies", type=int, default=AI_ENEMY_COUNT)
    parser.add_argument(
        "--vehicle-model", help="TC167 GLB with the existing hierarchy and optional BD clips"
    )
    parser.add_argument(
        "--no-gltf-animation", action="store_true", help="use the procedural animation fallback"
    )
    parser.add_argument("--ai-weapons", choices=("none", "guns", "all"), default="all")
    parser.add_argument("--atmosphere", action="store_true")
    parser.add_argument(
        "--model-test",
        action="store_true",
        help="display the continuous transformation inspection cycle",
    )
    parser.add_argument(
        "--prototype-test",
        action="store_true",
        help="automatic prototype pivot inspection with --model-test",
    )
    parser.add_argument(
        "--model-path", help="optional GLB for --model-test; no name-specific articulation"
    )
    parser.add_argument(
        "--inspect-model", help="print GLB hierarchy/bounds without creating a window"
    )
    parser.add_argument(
        "--vtol-test",
        choices=("ground", "landing", "skim", "hard", "crash"),
        help="VTOL ground/contact startup preset",
    )
    parser.add_argument(
        "--battledroid-test",
        choices=("ground", "landing", "skid"),
        help="Battledroid physical startup preset",
    )
    parser.add_argument(
        "--defense-test",
        choices=("radar", "ir", "mixed"),
        help="start with one or two valid pre-acquired hostile missile launches",
    )
    parser.add_argument(
        "--defense-range",
        type=float,
        default=1000.0,
        help="preset launch distance, 250-3500 meters",
    )
    parser.add_argument("--ai-ecm", action="store_true", help="enable ECM on alternating enemies")
    parser.add_argument(
        "--terrain-dir", help="directory of user-supplied HGT or WGS84 GeoTIFF elevation tiles"
    )
    parser.add_argument("--planet-tiles", type=Path, help="geographic asset registry directory")
    parser.add_argument("--world-manifest", type=Path)
    operating_mode = parser.add_mutually_exclusive_group()
    operating_mode.add_argument("--campaign", action="store_true", help="Start or resume Arrival")
    operating_mode.add_argument("--sandbox", action="store_true", help="Use the default sandbox")
    parser.add_argument(
        "--new-campaign", action="store_true", help="Replace an existing Arrival save"
    )
    parser.add_argument("--campaign-save", type=Path, help="Override the local Arrival save path")
    parser.add_argument("--planet-seed", type=int)
    parser.add_argument(
        "--terrain-patch-span",
        type=float,
        help="coarse streaming root span in meters",
    )
    parser.add_argument("--terrain-origin", nargs=2, type=float, metavar=("LAT", "LON"))
    parser.add_argument(
        "--terrain-offset", type=float, help="source EGM96 elevation assigned to world Y=0"
    )
    parser.add_argument("--terrain-budget", type=int)
    parser.add_argument("--terrain-view-distance", type=float)
    args = parser.parse_args()
    if not args.campaign and (args.new_campaign or args.campaign_save):
        parser.error("Campaign save options require --campaign")
    if args.campaign and any(
        (
            args.world_manifest,
            args.atmosphere,
            args.defense_test,
            args.vtol_test,
            args.battledroid_test,
            args.model_test,
            args.prototype_test,
            args.inspect_model,
            args.vehicle_model,
            args.legacy_animation,
            args.no_gltf_animation,
        )
    ):
        parser.error("Campaign mode uses the Arrival manifest and excludes sandbox presets")
    if args.inspect_model:
        from engine.gltf_loader import load_glb

        model = load_glb(args.inspect_model)
        model.print_hierarchy()
        print("Bounds:", model.bounds())
        return
    if args.vtol_test and args.battledroid_test:
        parser.error("--vtol-test and --battledroid-test are mutually exclusive")
    if args.prototype_test and (not args.model_test or args.model_path):
        parser.error("--prototype-test requires --model-test and excludes --model-path")
    if args.model_path and not args.model_test:
        parser.error("--model-path requires --model-test")
    if not 0 <= args.enemies <= 32:
        parser.error("--enemies must be between 0 and 32")
    if not 250 <= args.defense_range <= 3500:
        parser.error("--defense-range must be between 250 and 3500 meters")
    parameters = replace(
        AI,
        guns_enabled=args.ai_weapons != "none",
        missiles_enabled=args.ai_weapons == "all",
        ecm_enabled=args.ai_ecm,
    )
    from game.world_manifest import default_definition, load_manifest

    campaign = None
    try:
        if args.campaign:
            from game.arrival import DEFAULT_SAVE, open_campaign

            campaign = open_campaign(args.campaign_save or DEFAULT_SAVE, new=args.new_campaign)
        if campaign is not None:
            definition = campaign.definition
        else:
            definition = (
                load_manifest(args.world_manifest) if args.world_manifest else default_definition()
            )
        overrides = {
            "planetary_seed": args.planet_seed,
            "side": args.terrain_patch_span,
            "registry_directory": args.planet_tiles,
            "elevation_offset": args.terrain_offset,
            "budget": args.terrain_budget,
            "view_distance": args.terrain_view_distance,
            "directory": Path(args.terrain_dir) if args.terrain_dir else None,
        }
        overrides = {key: value for key, value in overrides.items() if value is not None}
        if args.terrain_origin:
            overrides["latitude"], overrides["longitude"] = args.terrain_origin
        if args.terrain_budget is not None:
            overrides["cache_size"] = max(definition.terrain.cache_size, args.terrain_budget + 1)
        terrain_config = replace(definition.terrain, **overrides)
    except (ValueError, OSError) as error:
        parser.error(str(error))
    if args.no_gltf_animation and not args.legacy_animation:
        parser.error("--no-gltf-animation requires --legacy-animation")
    game = Game(
        title=PROJECT_TITLE,
        enemy_count=args.enemies,
        ai_parameters=parameters,
        locomotion_animation=not args.no_locomotion_animation,
        terrain_config=terrain_config,
        world_manifest=definition,
        campaign=campaign,
        vehicle_model_path=args.vehicle_model,
        gltf_animation=not args.no_gltf_animation,
        legacy_animation=args.legacy_animation,
        animation_debug=args.animation_debug,
        anchor_debug=args.anchor_debug,
    )
    # Normal atmosphere startup remains in the hangar; F2 retains airborne recovery.
    if args.battledroid_test:
        game.setup_battledroid_test(args.battledroid_test)
    if args.vtol_test:
        game.setup_vtol_test(args.vtol_test)
    if args.defense_test:
        types = {
            "radar": (SeekerType.RADAR,),
            "ir": (SeekerType.IR,),
            "mixed": (SeekerType.RADAR, SeekerType.IR),
        }[args.defense_test]
        game.setup_defense_test(types, args.defense_range)
    if args.model_test:
        from game.model_test import ModelTest

        if args.prototype_test:
            from game.prototype_model import PrototypeTest

            game.world.model_test = PrototypeTest()
        else:
            if args.model_path:
                game.world.model_test = ModelTest(args.model_path)
            else:
                from game.transformation_test import TransformationTest

                game.world.model_test = TransformationTest()
        game.world.model_test.update(0.0)
    game.run()


if __name__ == "__main__":
    main()
