#!/usr/bin/env python3
"""Run the robot dashboard on this PC: python3 dashboard/run.py."""
import argparse
from pathlib import Path
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--record', action='store_true',
                        help='Record camera, debug, sensors, joint targets and servos to results/dashboard/.')
    parser.add_argument('--config', type=Path, default=Path(__file__).with_name('config.local.json'))
    parser.add_argument('--no-crest', action='store_true',
                        help='Keep the normal gait at crests; retain ramp tracking and camera changes.')
    crest = parser.add_mutually_exclusive_group()
    crest.add_argument('--crest-first', dest='crest_position', action='store_const', const='first',
                       help='Use the crest gait on the first climb (default).')
    crest.add_argument('--crest-last', dest='crest_position', action='store_const', const='last',
                       help='Use the crest gait on the second climb.')
    parser.set_defaults(crest_position='first')
    args = parser.parse_args()
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from robot_dashboard.runtime import Dashboard
    from robot_dashboard.server import DashboardServer
    recorder = None
    if args.record:
        from robot_dashboard.recording import DashboardRecording
        try:
            recorder = DashboardRecording()
        except RuntimeError as error:
            parser.error(str(error))
    app = Dashboard(args.config, crest_enabled=not args.no_crest, crest_position=args.crest_position)
    server = DashboardServer(('127.0.0.1', args.port), app)
    app.start()
    if recorder:
        app.camera.enable(True)
        recorder.start(f'http://127.0.0.1:{server.server_port}')
    print(f'Robot dashboard: http://127.0.0.1:{server.server_port}', flush=True)
    print('Keep the PC 2.4 GHz hotspot enabled. Ctrl+C stops the server.', flush=True)
    try:
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    finally:
        app.close()
        if recorder:
            recorder.close()
        server.server_close()


if __name__ == '__main__':
    main()
