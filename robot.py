"""Single entry point for B3 demos, training and checks. Run: python robot.py --help."""
import argparse
import os
import runpy
import sys
from robot_b3.paths import ROOT

COMMANDS = {
    'walk': ('robot_b3.run_robot_b3', 'Programmed walk; add --ramp for the full course'),
    'demo': ('robot_b3.meeting_demo', 'Meeting demo: normal, rl-flat, or rl-ramp'),
    'watch': ('robot_b3.watch_b3_rl', 'Watch or record a learned policy'),
    'train': ('robot_b3.train_b3_curriculum', 'Continue a flat / 3 / 5 degree curriculum lesson'),
    'train-flat': ('robot_b3.train_b3_rl', 'Start or continue the original flat PPO workflow'),
    'evaluate': ('robot_b3.evaluate_b3_rl', 'Evaluate an original flat-ground policy'),
    'build': ('robot_b3.build_robot_b3', 'Rebuild MuJoCo geometry from the CAD export'),
    'sensors': ('robot_b3.b3_sensors', 'Record sensor observations'),
    'record-course': ('robot_b3.record_b3_course', 'Record the programmed full-course walk'),
    'check-course': ('tests.test_b3_course', 'Run the longer gait/course experiment'),
}
CHECKS = {'rl': 'tests.test_b3_rl', 'curriculum': 'tests.test_b3_curriculum',
          'sensors': 'tests.test_b3_sensors', 'terrain': 'tests.test_b3_terrain',
          'structure': 'tests.test_structure'}

def main():
    # Resolve all existing relative checkpoint/config/output arguments against the repo.
    os.chdir(ROOT)
    args = sys.argv[1:]
    if not args or args[0] in ('-h', '--help'):
        print('Usage: python robot.py COMMAND [options]\n')
        for name, (_, description) in COMMANDS.items():
            print(f'  {name:15} {description}')
        print('  check           Run checks: all, rl, curriculum, sensors, terrain, structure')
        print('\nUse COMMAND --help for options. No training or simulation starts from this help screen.')
        return
    command, *rest = args
    if command == 'check':
        ap = argparse.ArgumentParser(prog='robot.py check')
        ap.add_argument('suite', nargs='?', default='all', choices=['all', *CHECKS])
        suite = ap.parse_args(rest).suite
        targets = CHECKS if suite == 'all' else {suite: CHECKS[suite]}
        for name, module in targets.items():
            print(f'Checking {name}...', flush=True)
            sys.argv = [module]
            runpy.run_module(module, run_name='__main__')
        return
    if command not in COMMANDS:
        raise SystemExit(f'Unknown command: {command}. Run python robot.py --help.')
    if command == 'build' and rest:
        if rest in (['--help'], ['-h']):
            print('Usage: python robot.py build\nRebuilds models from reference CAD exports; no options.')
            return
        raise SystemExit('build takes no arguments.')
    module = COMMANDS[command][0]
    sys.argv = [module, *rest]
    runpy.run_module(module, run_name='__main__')

if __name__ == '__main__':
    main()
