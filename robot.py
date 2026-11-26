"""B10 command entry point."""
import argparse
import os
import runpy
import sys
from robot_b91.paths import ROOT

COMMANDS = {
    'walk': ('robot_b91.run', 'Walking; --ramp or --corridor for a course, --test for report'),
    'crest': ('robot_b91.sim_crest', 'Headless trials of the crest gait on a foam-covered ramp'),
    'corridor': ('robot_b91.sim_corridor', 'Interactive B10 corridor; --test for a run without a viewer'),
    'build': ('robot_b91.build', 'Rebuild MuJoCo assets from the saved CAD export'),
}
CHECKS = {'b10': 'tests.test_b10', 'structure': 'tests.test_structure',
          'course': 'tests.test_course', 'walking': 'tests.test_walking',
          'transfer': 'tests.test_transfer'}

def main():
    os.chdir(ROOT)
    args = sys.argv[1:]
    if not args or args[0] in ('-h', '--help'):
        print('B10: python robot.py COMMAND [options]')
        for name, (_, description) in COMMANDS.items():
            print(f'  {name:10} {description}')
        print('  check      Run checks: all, b10, structure, course, walking, transfer')
        return
    command, *rest = args
    if command == 'walk' and '--corridor' in rest:
        if '--ramp' in rest:
            raise SystemExit('Choose either --ramp or --corridor.')
        command = 'corridor'
        rest = [arg for arg in rest if arg != '--corridor']
    if command == 'check':
        ap = argparse.ArgumentParser()
        ap.add_argument('suite', nargs='?', default='all', choices=['all', *CHECKS])
        suite = ap.parse_args(rest).suite
        suites = CHECKS if suite == 'all' else {suite: CHECKS[suite]}
        for name, module in suites.items():
            print(f'Checking {name}...', flush=True)
            sys.argv = [module]
            runpy.run_module(module, run_name='__main__')
        return
    if command not in COMMANDS:
        raise SystemExit(f'Unknown command: {command}. Run python robot.py --help.')
    if command == 'crest':
        # Worker processes must be able to import the trial function by module name.
        from robot_b91.sim_crest import main as run_crest
        raise SystemExit(run_crest(rest))
    if command == 'build' and rest:
        if rest in (['--help'], ['-h']):
            print('Usage: python robot.py build\nRebuild B10 from its saved CAD export.')
            return
        raise SystemExit('build takes no arguments.')
    module = COMMANDS[command][0]
    sys.argv = [module, *rest]
    runpy.run_module(module, run_name='__main__')

if __name__ == '__main__':
    main()
