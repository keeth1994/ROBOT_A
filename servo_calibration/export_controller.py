"""Export measured servos to the Arduino Wi-Fi controller's local header."""
import argparse
import json
from pathlib import Path

from calibrate import JOINTS, validate_measurements

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / 'firmware/servo_camera/controller_esp32/measured_servos.h'


def controller_header(data):
    validate_measurements(data)
    rows = []
    for joint in sorted(JOINTS, key=lambda name: data[name]['channel']):
        entry = data[joint]
        points = entry['points']
        pitch = joint.endswith('_pitch')
        values = [points[str(a)] for a in (-45, 0, 45)]
        values.append(points['-90'] if pitch else 0)
        rows.append(f'    {{"{joint}", {entry["channel"]}, {str(pitch).lower()}, '
                    + ', '.join(map(str, values)) + '},')
    return ('#pragma once\n#include "servo_angles.h"\n\nnamespace calibrated {\n'
            'constexpr Servo servos[] = {\n' + '\n'.join(rows) + '\n};\n'
            '}\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--file', type=Path, default=Path(__file__).with_name('measurements.json'))
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    header = controller_header(json.loads(args.file.read_text()))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(header)
    print(f'Wrote {args.output}')


if __name__ == '__main__':
    main()
