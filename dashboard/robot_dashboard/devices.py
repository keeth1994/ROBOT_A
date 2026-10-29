"""Small HTTP clients. No autonomous heartbeat or motion loop exists here."""
import http.client
import json
import re
from urllib.parse import urlsplit, urlencode


class DeviceError(Exception):
    def __init__(self, message, status=502):
        super().__init__(message)
        self.status = status


def origin(value):
    value = value.strip()
    if '://' not in value:
        value = 'http://' + value
    parsed = urlsplit(value)
    try:
        port = parsed.port
    except ValueError as error:
        raise ValueError('Invalid device port.') from error
    if (parsed.scheme != 'http' or not parsed.hostname or parsed.username or parsed.password or
            parsed.path not in ('', '/') or parsed.query or parsed.fragment):
        raise ValueError('Enter an HTTP device hostname or IP, with an optional port.')
    if not re.fullmatch(r'[A-Za-z0-9.\-:]+', parsed.hostname):
        raise ValueError('Invalid device hostname.')
    host = '[' + parsed.hostname + ']' if ':' in parsed.hostname else parsed.hostname
    return 'http://' + host + (':' + str(port) if port and port != 80 else '')


def stream_origin(camera, port=81):
    parsed = urlsplit(origin(camera))
    host = '[' + parsed.hostname + ']' if ':' in parsed.hostname else parsed.hostname
    return f'http://{host}:{port}'


def connection(base, timeout=0.8):
    parsed = urlsplit(base)
    # http.client deliberately avoids proxy environment variables for LAN devices.
    return http.client.HTTPConnection(parsed.hostname, parsed.port or 80, timeout=timeout)


def request(base, path, values=None, timeout=0.8):
    conn = connection(base, timeout)
    try:
        body = None if values is None else urlencode(values)
        headers = {} if body is None else {'Content-Type': 'application/x-www-form-urlencoded'}
        conn.request('GET' if values is None else 'POST', path, body, headers)
        response = conn.getresponse()
        payload = response.read(65537)
        if len(payload) > 65536:
            raise DeviceError('Device response exceeded the size limit.')
        text = payload.decode('utf-8', errors='replace')
        if response.status >= 300:
            try:
                text = json.loads(text).get('error', text)
            except (ValueError, AttributeError):
                pass
            raise DeviceError(text[:300] or f'Device HTTP {response.status}', response.status)
        if path in ('/status', '/settings', '/motion/start'):
            data = json.loads(text)
            if not isinstance(data, dict):
                raise DeviceError('Device returned an invalid status.')
            return data
        return {'message': text}
    except DeviceError:
        raise
    except (OSError, http.client.HTTPException, ValueError) as error:
        raise DeviceError(str(error)) from error
    finally:
        conn.close()


def mjpeg_frames(response, max_frame=2_000_000):
    """Parse bounded Content-Length multipart frames without queuing images."""
    buffer = bytearray()
    while True:
        part = response.read1(16384)
        if not part:
            raise DeviceError('Camera stream ended.')
        buffer.extend(part)
        while True:
            end = buffer.find(b'\r\n\r\n')
            if end < 0:
                if len(buffer) > 4096:
                    raise DeviceError('Invalid camera multipart header.')
                break
            match = re.search(br'(?im)^Content-Length:\s*(\d+)\s*\r?$', buffer[:end])
            if not match:
                raise DeviceError('Camera stream has no frame length.')
            length = int(match[1])
            if not 0 < length <= max_frame:
                raise DeviceError('Camera frame exceeded the size limit.')
            start = end + 4
            if len(buffer) < start + length:
                break
            jpeg = bytes(buffer[start:start + length])
            del buffer[:start + length]
            if not jpeg.startswith(b'\xff\xd8') or not jpeg.endswith(b'\xff\xd9'):
                raise DeviceError('Camera stream contains an invalid JPEG frame.')
            yield jpeg
        if len(buffer) > max_frame + 4096:
            raise DeviceError('Camera stream buffer exceeded the size limit.')
