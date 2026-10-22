"""Loopback-only dashboard HTTP server. All motion writes are explicit requests."""
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import secrets
import time
from urllib.parse import urlsplit

from .devices import DeviceError

STATIC = Path(__file__).resolve().parents[1] / 'static'


class DashboardServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, app):
        self.app = app
        self.token = secrets.token_urlsafe(32)
        super().__init__(address, Handler)


class Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'

    def log_message(self, fmt, *args):
        pass

    def json(self, data, status=200):
        self.send_bytes(json.dumps(data, allow_nan=False).encode(), 'application/json', status)

    def send_bytes(self, data, content_type, status=200):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.end_headers()
        self.wfile.write(data)

    def valid_host(self):
        return self.headers.get('Host') in (f'127.0.0.1:{self.server.server_port}', f'localhost:{self.server.server_port}')

    def do_GET(self):
        if not self.valid_host():
            return self.json({'error': 'Use the localhost dashboard address.'}, 403)
        path = urlsplit(self.path).path
        try:
            if path == '/api/state':
                return self.json({**self.server.app.state(), 'token': self.server.token})
            if path == '/api/video.mjpeg':
                return self.stream()
            files = {'/': ('index.html', 'text/html; charset=utf-8'),
                     '/app.js': ('app.js', 'text/javascript'), '/styles.css': ('styles.css', 'text/css')}
            if path not in files:
                return self.json({'error': 'Not found.'}, 404)
            filename, content_type = files[path]
            return self.send_bytes((STATIC / filename).read_bytes(), content_type)
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass

    def do_POST(self):
        expected_origin = 'http://' + self.headers.get('Host', '')
        supplied = self.headers.get('X-Dashboard-Token', '')
        if (not self.valid_host() or self.headers.get('Origin', expected_origin) != expected_origin or
                not hmac.compare_digest(supplied, self.server.token)):
            return self.json({'error': 'Reload the local dashboard before sending commands.'}, 403)
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= 8192:
                raise ValueError('Invalid request length.')
            self.connection.settimeout(3)
            values = json.loads(self.rfile.read(length))
            if not isinstance(values, dict):
                raise ValueError('Expected a JSON object.')
            path = urlsplit(self.path).path
            app = self.server.app
            if path == '/api/heartbeat':
                result = app.servo_command('/heartbeat', {})
            elif path in ('/api/servo/start', '/api/servo/servo', '/api/servo/release', '/api/servo/stop'):
                result = app.servo_command('/' + path.rsplit('/', 1)[1], values)
            elif path == '/api/config':
                result = app.configure(values)
            elif path == '/api/video':
                if not isinstance(values.get('enabled'), bool):
                    raise ValueError('Expected a video enabled flag.')
                app.camera.enable(values['enabled'])
                result = {'enabled': values['enabled']}
            elif path == '/api/camera/settings':
                if (values.get('resolution') not in ('qvga', 'vga', 'svga') or
                        values.get('fps') not in (5, 10, 15) or values.get('quality') not in (12, 18, 24)):
                    raise ValueError('Unsupported camera settings.')
                result = app.camera_settings(values)
            else:
                return self.json({'error': 'Unknown command.'}, 404)
            self.json(result)
        except DeviceError as error:
            self.json({'error': str(error)}, error.status if 400 <= error.status <= 599 else 502)
        except (ValueError, KeyError, TypeError) as error:
            self.json({'error': str(error)}, 400)
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass

    def stream(self):
        self.connection.settimeout(2)
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=pc-frame')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Connection', 'close')
        self.end_headers()
        self.close_connection = True
        sequence = -1
        while not self.server.app.stop_event.is_set():
            if not self.server.app.camera.enabled:
                return
            frame = self.server.app.camera.frame(sequence)
            if frame:
                sequence, jpeg = frame
                header = f'\r\n--pc-frame\r\nContent-Type: image/jpeg\r\nContent-Length: {len(jpeg)}\r\n\r\n'.encode()
                self.wfile.write(header + jpeg)
                self.wfile.flush()
            else:
                time.sleep(.025)
