#pragma once
#include "control.h"

namespace hw {
// Proposed ESP32 GPIOs, not pin numbers from the supplied wiring diagram.
constexpr int sda = 21, scl = 22;
constexpr uint8_t servoAddress = 0x40, imuAddress = 0x68;
constexpr double pwmOscillatorHz = 25000000; // Measure if pulse timing differs.

// Keep false until GPIOs, IMU axes, servo channels and PWM calibration are checked.
constexpr bool calibrated = false;
// User-supplied wiring: FL yaw/pitch, RL yaw/pitch, FR yaw/pitch, RR yaw/pitch.
// Pulse values remain unset until each joint is measured.
// Pitch needs five samples in raised frame (+45 = CAD neutral).
// C++ field order: channel, -45, 0, +45, +90, -90.
constexpr std::array<b91::ServoCalibration, 8> servos{{
    {8, 0, 0, 0}, {3, 0, 0, 0}, {1, 0, 0, 0}, {2, 0, 0, 0},
    {0, 0, 0, 0}, {11, 0, 0, 0}, {9, 0, 0, 0}, {10, 0, 0, 0}
}};
// Sensor-to-body rotation: body +X forward, +Y left, +Z up. Verify mounting.
constexpr double imuRotation[3][3] = {{1,0,0}, {0,1,0}, {0,0,1}};
// Raw pulse commissioning is separate from walking. Enable for a supported bench robot.
constexpr bool benchCalibration = false;
constexpr int benchMinUs = 1400, benchMaxUs = 1600;
} // namespace hw
