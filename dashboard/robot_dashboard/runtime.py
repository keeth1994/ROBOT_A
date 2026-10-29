import http.client
import json
from pathlib import Path
import threading
import time

from . import devices
from .walking import Walking


class CameraPipeline:
    def __init__(self, get_stream_url):
        self.get_stream_url = get_stream_url
        self.condition = threading.Condition()
        self.stop_event = threading.Event()
        self.enabled = False
        self.generation = 0
        self.sequence = 0
        self.raw = None
        self.received_at = 0
        self.error = 'Video is off.'
        self.conn = self.response = None
        self.receive_times = []
        self.threads = []

    def start(self):
        thread = threading.Thread(target=self._capture, daemon=True)
        thread.start()
        self.threads.append(thread)

    def enable(self, enabled):
        with self.condition:
            self.enabled = enabled
            self.generation += 1
            self.raw = None
            self.received_at = 0
            self.receive_times.clear()
            self.error = 'Connecting to camera...' if enabled else 'Video is off.'
            conn, response = self.conn, self.response
            self.condition.notify_all()
        # Close outside the condition; an upstream read has a bounded timeout.
        if conn:
            conn.close()
        if response:
            try:
                response.close()
            except OSError:
                pass

    def close(self):
        self.stop_event.set()
        self.enable(False)
        for thread in self.threads:
            thread.join(timeout=3)

    def _capture(self):
        while not self.stop_event.is_set():
            with self.condition:
                self.condition.wait_for(lambda: self.enabled or self.stop_event.is_set(), timeout=.5)
                if not self.enabled or self.stop_event.is_set():
                    continue
                generation = self.generation
            conn = response = None
            try:
                conn = devices.connection(self.get_stream_url(), timeout=2)
                conn.request('GET', '/stream')
                response = conn.getresponse()
                if response.status != 200:
                    raise devices.DeviceError(f'Camera stream HTTP {response.status}')
                with self.condition:
                    if generation != self.generation:
                        continue
                    self.conn, self.response = conn, response
                for jpeg in devices.mjpeg_frames(response):
                    with self.condition:
                        if generation != self.generation or not self.enabled or self.stop_event.is_set():
                            break
                        now = time.monotonic()
                        self.raw = jpeg
                        self.sequence += 1
                        self.received_at = now
                        self.receive_times = [t for t in self.receive_times if now - t < 2] + [now]
                        self.error = ''
                        self.condition.notify_all()
            except (OSError, http.client.HTTPException, ValueError, devices.DeviceError, AttributeError) as error:
                with self.condition:
                    if generation == self.generation:
                        self.error = str(error)
                        self.raw = None
                        self.condition.notify_all()
            finally:
                if response:
                    response.close()
                if conn:
                    conn.close()
                with self.condition:
                    if self.conn is conn:
                        self.conn = self.response = None
            self.stop_event.wait(.5)

    def state(self):
        with self.condition:
            now = time.monotonic()
            fresh = self.raw is not None and self.received_at > 0 and now - self.received_at < 1.5
            times = self.receive_times
            fps = (len(times) - 1) / (times[-1] - times[0]) if fresh and len(times) > 1 and times[-1] > times[0] else 0
            return {'enabled': self.enabled, 'fresh': fresh,
                    'receive_fps': round(fps, 1),
                    'receive_age_ms': round((now - self.received_at) * 1000) if self.received_at else None,
                    'error': self.error}

    def frame(self, after=-1):
        with self.condition:
            sequence, jpeg, received = self.sequence, self.raw, self.received_at
            if not self.enabled or not jpeg or time.monotonic() - received >= 1.5 or sequence == after:
                return None
            return sequence, jpeg


class Dashboard:
    def __init__(self, config_path=None):
        self.lock = threading.RLock()
        self.servo_lock = threading.Lock()
        self.camera_settings_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.config_path = Path(config_path) if config_path else None
        self.config = {'servo': 'http://robot-a-servo.local', 'camera': 'http://robot-a-camera.local', 'stream_port': 81}
        if self.config_path and self.config_path.exists():
            self.config = self._validate_config(json.loads(self.config_path.read_text()))
        self.generation = 0
        self.status_epoch = {'servo': 0, 'camera': 0}
        self.browser_heartbeat = 0
        self.cache = {name: {'online': False, 'data': None, 'error': 'Connecting...', 'at': 0}
                      for name in ('servo', 'camera')}
        self.camera = CameraPipeline(self.stream_url)
        self.walking = Walking()
        self.threads = []

    @staticmethod
    def _validate_config(values):
        port = values.get('stream_port', 81)
        if not isinstance(port, int) or isinstance(port, bool) or not 1 <= port <= 65535:
            raise ValueError('Camera stream port must be 1–65535.')
        return {'servo': devices.origin(values['servo']), 'camera': devices.origin(values['camera']), 'stream_port': port}

    def stream_url(self):
        with self.lock:
            return devices.stream_origin(self.config['camera'], self.config['stream_port'])

    def start(self):
        self.camera.start()
        for name in ('servo', 'camera'):
            thread = threading.Thread(target=self._poll, args=(name,), daemon=True)
            thread.start()
            self.threads.append(thread)
        thread = threading.Thread(target=self._walk_loop, daemon=True)
        thread.start()
        self.threads.append(thread)

    def _walk_loop(self):
        while not self.stop_event.wait(.02):
            with self.servo_lock:
                if not self.walking.active:
                    continue
                with self.lock:
                    alive = time.monotonic() - self.browser_heartbeat < 1.5
                    url = self.config['servo']
                try:
                    if not alive:
                        raise devices.DeviceError('Operator heartbeat lost; walking stopped.')
                    status = devices.request(url, '/status', timeout=.15)
                    frame = self.walking.frame(status, time.monotonic())
                    devices.request(url, '/targets', frame, timeout=.15)
                except (devices.DeviceError, ValueError, KeyError, TypeError) as error:
                    self.walking.stop(str(error))
                    try:
                        devices.request(url, '/stop', {}, timeout=.15)
                    except devices.DeviceError:
                        pass

    def walk_command(self, action):
        with self.servo_lock:
            with self.lock:
                alive = time.monotonic() - self.browser_heartbeat < 1.5
                url = self.config['servo']
            if action != 'start':
                raise ValueError('Unknown walking command.')
            if not alive:
                raise devices.DeviceError('Keep the dashboard visible to start walking.', 409)
            if self.walking.active:
                raise devices.DeviceError('Already walking.', 409)
            try:
                status = devices.request(url, '/status', timeout=.15)
                self.walking.prepare(status)
                if status.get('manualActive') or status.get('running') or status.get('externalActive'):
                    raise devices.DeviceError('Press Stop to release existing servo control first.', 409)
                result = devices.request(url, '/motion/start', {}, timeout=.15)
                self.walking.begin(status, result['session'], time.monotonic())
            except (devices.DeviceError, KeyError, ValueError, TypeError) as error:
                self.walking.stop('Walking start failed. Outputs off requested.')
                try:
                    devices.request(url, '/stop', {}, timeout=.15)
                except devices.DeviceError:
                    pass
                if isinstance(error, devices.DeviceError):
                    raise
                raise devices.DeviceError('Invalid ESP32 walking response; motion stopped.', 409) from error
            return {'message': self.walking.message}

    def _poll(self, name):
        while not self.stop_event.is_set():
            with self.lock:
                url, generation = self.config[name], self.generation
                epoch = self.status_epoch[name]
            try:
                data = devices.request(url, '/status')
                if name == 'servo' and 'servoReady' not in data or name == 'camera' and 'ready' not in data:
                    raise devices.DeviceError('Wrong device or firmware at this address.')
                update = {'online': True, 'data': data, 'error': '', 'at': time.monotonic()}
            except devices.DeviceError as error:
                update = {'online': False, 'data': None, 'error': str(error), 'at': 0}
            with self.lock:
                if generation == self.generation and epoch == self.status_epoch[name]:
                    self.cache[name] = update
            self.stop_event.wait(.5 if name == 'servo' else 1)

    def state(self):
        with self.lock:
            now = time.monotonic()
            status = {'config': dict(self.config)}
            for name, entry in self.cache.items():
                status[name] = {**entry, 'online': entry['online'] and now - entry['at'] < 2.5}
            status['operator_alive'] = now - self.browser_heartbeat < 1.5
        status['pipeline'] = self.camera.state()
        status['walking'] = self.walking.state()
        return status

    def servo_command(self, path, values):
        if path not in ('/heartbeat', '/start', '/servo', '/release', '/stop'):
            raise ValueError('Unknown servo command.')
        arrival = time.monotonic()
        with self.lock:
            generation = self.generation
        with self.servo_lock:
            if path == '/stop':
                self.walking.stop()
            elif self.walking.active and path not in ('/heartbeat',):
                raise devices.DeviceError('Stop walking before manual control or sweeps.', 409)
            with self.lock:
                if generation != self.generation:
                    raise devices.DeviceError('Device address changed. Retry explicitly.', 409)
                if path in ('/start', '/servo'):
                    now = time.monotonic()
                    if now - arrival > 1.5 or now - self.browser_heartbeat > 1.5:
                        raise devices.DeviceError('Operator heartbeat expired. Keep the dashboard visible.', 409)
                    entry = self.cache['servo']
                    if not entry['online'] or now - entry['at'] > 2.5:
                        raise devices.DeviceError('Controller status is unavailable.', 409)
                url = self.config['servo']
            result = devices.request(url, path, values)
            if path == '/heartbeat':
                # This happens ONLY in response to a browser request, never a timer.
                with self.lock:
                    self.browser_heartbeat = arrival
            return result

    def configure(self, values):
        config = self._validate_config(values)
        # Operator-requested address changes cancel current control before switching.
        with self.camera_settings_lock, self.servo_lock:
            was_walking = self.walking.active
            self.walking.stop('Device address changed. Walking stopped.')
            with self.lock:
                active = time.monotonic() - self.browser_heartbeat < 1.5
                data = self.cache['servo']['data'] or {}
                active = active or was_walking or data.get('running', False) or data.get('manualActive', False) or data.get('externalActive', False)
                old_url = self.config['servo']
            if active:
                devices.request(old_url, '/stop', {})
            self.camera.enable(False)
            with self.lock:
                self.generation += 1
                self.browser_heartbeat = 0
                self.config = config
                self.cache = {name: {'online': False, 'data': None, 'error': 'Connecting...', 'at': 0}
                              for name in ('servo', 'camera')}
            if self.config_path:
                self.config_path.write_text(json.dumps(config, indent=2) + '\n')
        return config

    def camera_settings(self, values):
        with self.camera_settings_lock:
            with self.lock:
                self.status_epoch['camera'] += 1
            resume = self.camera.enabled
            self.camera.enable(False)
            with self.lock:
                url = self.config['camera']
            for attempt in range(16):
                try:
                    result = devices.request(url, '/settings', values, timeout=2)
                    break
                except devices.DeviceError as error:
                    if error.status != 409 or attempt == 15:
                        raise
                    time.sleep(.15)
            with self.lock:
                self.status_epoch['camera'] += 1
                self.cache['camera'] = {'online': True, 'data': result, 'error': '', 'at': time.monotonic()}
            if resume:
                self.camera.enable(True)
            return result

    def close(self):
        self.stop_event.set()
        with self.lock:
            data = self.cache['servo']['data'] or {}
            active = self.walking.active or time.monotonic() - self.browser_heartbeat < 3 or data.get('running') or data.get('manualActive') or data.get('externalActive')
        if active:
            try:
                self.servo_command('/stop', {})
            except devices.DeviceError:
                pass  # MCU watchdog remains the final fallback if the PC/link is gone.
        self.camera.close()
        for thread in self.threads:
            thread.join(timeout=2)
