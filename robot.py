"""B9-1 command entry point."""
import argparse
import os
import runpy
import sys
from robot_b91.paths import ROOT

COMMANDS = {
    'walk': ('robot_b91.run', 'Programmed walking; --ramp for course, --test for report'),
    'build': ('robot_b91.build', 'Rebuild MuJoCo assets from the saved CAD export'),
}
CHECKS = {'b91': 'tests.test_b91', 'structure': 'tests.test_structure'}

def main():
    os.chdir(ROOT)
    args = sys.argv[1:]
    if not args or args[0] in ('-h', '--help'):
        print('B9-1: python robot.py COMMAND [options]')
        for name, (_, description) in COMMANDS.items():
            print(f'  {name:10} {description}')
        print('  check      Run checks: all, b91, structure')
        return
    command, *rest = args
    if command == 'check':
        ap = argparse.ArgumentParser()
        ap.add_argument('suite', nargs='?', default='all', choices=['all', *CHECKS])
        suite = ap.parse_args(rest).suite
        for name, module in (CHECKS if suite == 'all' else {suite: CHECKS[suite]}).items():
            print(f'Checking {name}...', flush=True)
            sys.argv = [module]
            runpy.run_module(module, run_name='__main__')
        return
    if command not in COMMANDS:
        raise SystemExit(f'Unknown command: {command}. Run python robot.py --help.')
    if command == 'build' and rest:
        if rest in (['--help'], ['-h']):
            print('Usage: python robot.py build\nRebuild B9-1 from its saved CAD export.')
            return
        raise SystemExit('build takes no arguments.')
    module = COMMANDS[command][0]
    sys.argv = [module, *rest]
    runpy.run_module(module, run_name='__main__')

if __name__ == '__main__':
    main()
