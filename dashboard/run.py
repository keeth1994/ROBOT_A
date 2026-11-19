#!/usr/bin/env python3
"""Run the robot dashboard on this PC: python3 dashboard/run.py."""
import argparse
from pathlib import Path
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--config', type=Path, default=Path(__file__).with_name('config.local.json'))
    parser.add_argument('--ramp-only', action='store_true',
                        help='Stop after the ramp instead of automatically starting corridor search.')
    parser.add_argument('--crest-assist', action='store_true',
                        help='Experiment: switch gait at the first crest using onboard pitch.')
    parser.add_argument('--crest-wave', action='store_true',
                        help='Experiment: single-foot crest gait at requested speed; requires updated firmware. Implies crest assist and adaptive speed.')
    parser.add_argument('--exit-clearance', action='store_true',
                        help='Clear the rear feet after ramp exit detection; also enabled by --crest-assist.')
    parser.add_argument('--adaptive-ramp-speed', action='store_true',
                        help='Limit speed to 50%% when roll, heading error or lateral drift increases.')
    args = parser.parse_args()
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from robot_dashboard.runtime import Dashboard
    from robot_dashboard.server import DashboardServer
    app = Dashboard(args.config, auto_corridor=not args.ramp_only,
                    crest_assist=args.crest_assist, exit_clearance=args.exit_clearance,
                    adaptive_ramp_speed=args.adaptive_ramp_speed, crest_wave=args.crest_wave)
    server = DashboardServer(('127.0.0.1', args.port), app)
    app.start()
    print(f'Robot dashboard: http://127.0.0.1:{server.server_port}', flush=True)
    print('Keep the PC 2.4 GHz hotspot enabled. Ctrl+C stops the server.', flush=True)
    try:
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    finally:
        app.close()
        server.server_close()


if __name__ == '__main__':
    main()
