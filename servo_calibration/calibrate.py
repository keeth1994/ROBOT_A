import argparse
import json
import queue
import threading
import time
from pathlib import Path

JOINTS = ('FL_yaw', 'FL_pitch', 'RL_yaw', 'RL_pitch',
          'FR_yaw', 'FR_pitch', 'RR_yaw', 'RR_pitch')


def angles_for(joint):
    # Pitch calibration zero is 45 degrees above CAD neutral.
    return (-90, -45, 0, 45, 90) if joint.endswith('_pitch') else (-45, 0, 45)


def export_table(data):
    rows, used = [], set()
    for joint in JOINTS:
        entry = data[joint]
        channel = entry['channel']
        points = entry['points']
        values = [points[str(a)] for a in angles_for(joint)]
        if channel not in (0, 1, 2, 3, 8, 9, 10, 11) or channel in used:
            raise ValueError('Each joint needs a distinct configured channel')
        if any(not 500 <= v <= 2500 for v in values):
            raise ValueError('Pulse outside electrical bounds')
        differences = [b - a for a, b in zip(values, values[1:])]
        if any(abs(d) < 20 or (d > 0) != (differences[0] > 0) for d in differences):
            raise ValueError(f'{joint}: measurements must be monotonic, >=20 us apart')
        used.add(channel)
        # Preserve the C++ field order: channel, -45, 0, +45, +90, -90.
        field_angles = (-45, 0, 45, 90, -90) if joint.endswith('_pitch') else (-45, 0, 45)
        fields = (channel, *(points[str(a)] for a in field_angles))
        rows.append('    {' + ', '.join(map(str, fields)) + f'}}, // {joint}')
    return ('// Yaw: -45, 0, +45 degrees relative to CAD.\n'
            '// Pitch fields: -45, 0, +45, +90, -90 degrees relative to raised zero.\n'
            '// Pitch +45 = CAD neutral; -45 = horizontal outward.\n'
            'constexpr std::array<b91::ServoCalibration, 8> servos{{\n'
            + '\n'.join(rows) + '\n}};\n')


class Connection:
    def __init__(self, port):
        import serial
        self.port = serial.Serial(port, 115200, timeout=.1, write_timeout=.5)
        time.sleep(2)  # ESP32 may reset when the port opens.
        self.port.reset_input_buffer()
        self.lock = threading.Lock()
        self.replies = queue.Queue()
        self.stop = threading.Event()
        self.worker = threading.Thread(target=self.service, daemon=True)
        self.worker.start()

    def send(self, text):
        with self.lock:
            self.port.write((text + '\n').encode('ascii'))

    def service(self):
        next_ping = 0
        try:
            while not self.stop.is_set():
                if time.monotonic() >= next_ping:
                    self.send('PING')
                    next_ping = time.monotonic() + .3
                line = self.port.readline().decode('ascii', errors='replace').strip()
                if line and line != 'OK PING':
                    self.replies.put(line)
        except Exception as exc:
            self.replies.put(f'ERR connection lost: {exc}')

    def request(self, text):
        # A command is considered applied only after its firmware acknowledgement.
        self.send(text)
        reply = self.replies.get(timeout=2)
        if not reply.startswith('OK '):
            raise ValueError(reply)
        return reply

    def close(self):
        try:
            self.request('OFF')
        finally:
            self.stop.set()
            self.worker.join(timeout=1)
            self.port.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('port', help='e.g. /dev/ttyUSB0 or COM3')
    parser.add_argument('--file', type=Path, default=Path(__file__).with_name('measurements.json'))
    args = parser.parse_args()
    data = json.loads(args.file.read_text()) if args.file.exists() else {}
    connection = Connection(args.port)
    joint = None
    pulse = None
    channel = None
    print('Support robot; SELECT moves a servo to 1500 us. Other outputs switch off.')
    print('Commands: select JOINT CHANNEL | jog +/-US (max 10) | record ANGLE')
    print('Yaw: -45/0/+45 CAD. Pitch: -90/-45/0/+45/+90 raised frame; +45 = CAD neutral.')
    print('          off | status | export | quit')
    print('Joints:', ', '.join(JOINTS))
    try:
        print(connection.request('STATUS'))
        while True:
            try:
                words = input('cal> ').split()
                if not words:
                    continue
                command, *rest = words
                if command == 'quit':
                    break
                if command == 'select' and len(rest) == 2:
                    name, raw_channel = rest
                    if name not in JOINTS:
                        raise ValueError('Unknown joint')
                    new_channel = int(raw_channel)
                    if name in data and data[name]['channel'] != new_channel:
                        raise ValueError('Saved joint uses another channel; correct the measurements file first')
                    reply = connection.request(f'SELECT {new_channel}')
                    _, _, actual_channel, actual_pulse = reply.split()
                    joint, channel, pulse = name, int(actual_channel), int(actual_pulse)
                    print(reply)
                elif command == 'jog' and len(rest) == 1:
                    if joint is None:
                        raise ValueError('Select a joint first')
                    reply = connection.request(f'JOG {int(rest[0])}')
                    _, _, actual_channel, actual_pulse = reply.split()
                    if int(actual_channel) != channel:
                        raise ValueError('Unexpected active channel')
                    pulse = int(actual_pulse)
                    print(reply)
                elif command == 'record' and len(rest) == 1:
                    angle = int(rest[0])
                    if joint is None or angle not in angles_for(joint):
                        raise ValueError('Measure yaw -45/0/+45 or pitch -90/-45/0/+45/+90 degrees')
                    status = connection.request('STATUS')
                    fields = dict(item.split('=') for item in status.split()[2:])
                    if int(fields['channel']) != channel or int(fields['pulse']) != pulse:
                        raise ValueError('Servo state changed; select joint again')
                    data.setdefault(joint, {'channel': channel, 'points': {}})['points'][str(angle)] = pulse
                    args.file.parent.mkdir(parents=True, exist_ok=True)
                    temporary = args.file.with_suffix('.tmp')
                    temporary.write_text(json.dumps(data, indent=2) + '\n')
                    temporary.replace(args.file)
                    print(f'Saved {joint}: measured {angle} degrees at {pulse} us')
                elif command == 'export' and not rest:
                    output = args.file.with_name('servo_table.h')
                    output.write_text(export_table(data))
                    print(f'Wrote {output}')
                elif command in ('off', 'status') and not rest:
                    print(connection.request(command.upper()))
                    if command == 'off':
                        joint = pulse = channel = None
                else:
                    print('Unknown command or incorrect arguments')
            except (ValueError, KeyError, queue.Empty) as exc:
                # Unknown command outcome: release outputs and require selection again.
                joint = pulse = channel = None
                connection.send('OFF')
                print(f'Error: {exc}. Outputs requested off; select joint again.')
                break
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        connection.close()


if __name__ == '__main__':
    main()
