import json

from robot.paths import ROOT


def servo_to_cradle_deg(angle, settings):
    fraction = ((angle - settings['forward_servo_deg']) /
                (settings['backward_servo_deg'] - settings['forward_servo_deg']))
    return settings['forward_deg'] + fraction * (settings['backward_deg'] - settings['forward_deg'])


def cradle_to_servo_deg(angle, settings):
    fraction = ((angle - settings['forward_deg']) /
                (settings['backward_deg'] - settings['forward_deg']))
    return settings['forward_servo_deg'] + fraction * (settings['backward_servo_deg'] - settings['forward_servo_deg'])


def load_camera_scan_settings():
    settings = json.loads((ROOT / 'reference/b10_camera_scan.json').read_text())
    if settings['forward_servo_deg'] == settings['backward_servo_deg']:
        raise ValueError('Forward and backward camera commands must differ.')
    return settings
