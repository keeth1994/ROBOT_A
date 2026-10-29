#!/usr/bin/env python3
"""Run the robot dashboard on this PC: python3 dashboard/run.py."""
import argparse
from pathlib import Path
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--config', type=Path, default=Path(__file__).with_name('config.local.json'))
    args = parser.parse_args()
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from robot_dashboard.runtime import Dashboard
    from robot_dashboard.server import DashboardServer
    app = Dashboard(args.config)
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
