"""Project Tango Charlie entry point and reproducible AI test presets."""
import argparse
from dataclasses import replace
from engine.game import Game
from game.identity import PROJECT_TITLE
from game.ai_config import AI, AI_ENEMY_COUNT
from game.missile_seeker import SeekerType


def main():
    parser = argparse.ArgumentParser(description=PROJECT_TITLE)
    parser.add_argument('--no-locomotion-animation', action='store_true', help='disable visual Battledroid gait; physics unchanged')
    parser.add_argument('--enemies', type=int, default=AI_ENEMY_COUNT)
    parser.add_argument('--ai-weapons', choices=('none','guns','all'), default='all')
    parser.add_argument('--atmosphere', action='store_true')
    parser.add_argument('--model-test', action='store_true', help='display the continuous transformation inspection cycle')
    parser.add_argument('--prototype-test', action='store_true', help='automatic prototype pivot inspection with --model-test')
    parser.add_argument('--model-path', help='optional GLB for --model-test; no name-specific articulation')
    parser.add_argument('--inspect-model', help='print GLB hierarchy/bounds without creating a window')
    parser.add_argument('--vtol-test', choices=('ground','landing','skim','hard','crash'), help='VTOL ground/contact startup preset')
    parser.add_argument('--battledroid-test', choices=('ground','landing','skid'), help='Battledroid physical startup preset')
    parser.add_argument('--defense-test', choices=('radar','ir','mixed'),
                        help='start with one or two valid pre-acquired hostile missile launches')
    parser.add_argument('--defense-range', type=float, default=1000., help='preset launch distance, 250-3500 meters')
    parser.add_argument('--ai-ecm', action='store_true', help='enable ECM on alternating enemies')
    parser.add_argument('--terrain-dir', help='directory of user-supplied uncompressed HGT tiles')
    parser.add_argument('--terrain-origin', nargs=2, type=float, metavar=('LAT', 'LON'), default=(34.5,-111.5))
    parser.add_argument('--terrain-offset', type=float, help='source EGM96 elevation assigned to world Y=0')
    parser.add_argument('--terrain-budget', type=int, default=64)
    parser.add_argument('--terrain-view-distance', type=float, default=16000.)
    args = parser.parse_args()
    if args.inspect_model:
        from engine.gltf_loader import load_glb
        model=load_glb(args.inspect_model);model.print_hierarchy();print('Bounds:',model.bounds())
        return
    if args.vtol_test and args.battledroid_test:parser.error('--vtol-test and --battledroid-test are mutually exclusive')
    if args.prototype_test and (not args.model_test or args.model_path):parser.error('--prototype-test requires --model-test and excludes --model-path')
    if args.model_path and not args.model_test:parser.error('--model-path requires --model-test')
    if not 0 <= args.enemies <= 32:
        parser.error('--enemies must be between 0 and 32')
    if not 250 <= args.defense_range <= 3500:
        parser.error('--defense-range must be between 250 and 3500 meters')
    parameters = replace(AI, guns_enabled=args.ai_weapons != 'none',
                         missiles_enabled=args.ai_weapons == 'all',ecm_enabled=args.ai_ecm)
    from game.terrain import TerrainConfig
    from pathlib import Path
    try:
        terrain_config = TerrainConfig(latitude=args.terrain_origin[0], longitude=args.terrain_origin[1],
            elevation_offset=args.terrain_offset, budget=args.terrain_budget,
            cache_size=max(96,args.terrain_budget+1), view_distance=args.terrain_view_distance,
            **({'directory': Path(args.terrain_dir)} if args.terrain_dir else {}))
    except ValueError as error:
        parser.error(str(error))
    game = Game(title=PROJECT_TITLE, enemy_count=args.enemies, ai_parameters=parameters,
                locomotion_animation=not args.no_locomotion_animation, terrain_config=terrain_config)
    if args.atmosphere:
        game.input.atmosphere_pressed = True
        game.update(0)
        game.input.atmosphere_pressed = False
    if args.battledroid_test:
        game.setup_battledroid_test(args.battledroid_test)
    if args.vtol_test:
        game.setup_vtol_test(args.vtol_test)
    if args.defense_test:
        types={'radar':(SeekerType.RADAR,), 'ir':(SeekerType.IR,),
               'mixed':(SeekerType.RADAR,SeekerType.IR)}[args.defense_test]
        game.setup_defense_test(types,args.defense_range)
    if args.model_test:
        from game.model_test import ModelTest
        if args.prototype_test:
            from game.prototype_model import PrototypeTest
            game.world.model_test = PrototypeTest()
        else:
            if args.model_path:
                game.world.model_test=ModelTest(args.model_path)
            else:
                from game.transformation_test import TransformationTest
                game.world.model_test=TransformationTest()
        game.world.model_test.update(0.)
    game.run()


if __name__ == '__main__':
    main()
