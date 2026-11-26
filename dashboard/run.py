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
    parser.add_argument('--start-at-crest', action='store_true',
                        help='Assisted placement test: begin in the crest wave gait. Requires --ramp-only.')
    parser.add_argument('--exit-clearance', action='store_true',
                        help='Clear the rear feet after ramp exit detection; enabled by default for ramp runs.')
    parser.add_argument('--adaptive-ramp-speed', action='store_true',
                        help='Limit speed to 50%% when roll, heading error or lateral drift increases; enabled by default for ramp runs.')
    parser.add_argument('--ramp-drift-control', action='store_true',
                        help='Correct sideways earlier and shorten forward stride when ramp offset grows.')
    parser.add_argument('--control-log', type=Path,
                        help='Record control updates and corresponding onboard frames to a new JSONL file.')
    args = parser.parse_args()
    if args.start_at_crest and not args.ramp_only:
        parser.error('--start-at-crest requires --ramp-only.')
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from robot_dashboard.runtime import Dashboard
    from robot_dashboard.server import DashboardServer
    app = Dashboard(args.config, auto_corridor=not args.ramp_only,
                    exit_clearance=args.exit_clearance,
                    adaptive_ramp_speed=args.adaptive_ramp_speed,
                    start_at_crest=args.start_at_crest,
                    ramp_drift_control=args.ramp_drift_control, control_log=args.control_log)
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
