#pragma once
#include "servo_angles.h"

namespace camera_pitch {
constexpr int channel = 14;
constexpr int minAngle = -45, maxAngle = 45;
constexpr calibrated::Servo servo = {"camera_pitch", channel, true, 970, 1500, 1890, 550};
inline int pulse(double angle) {
  if (!std::isfinite(angle) || angle < minAngle || angle > maxAngle) return -1;
  return calibrated::pulse(servo, angle);
}
}
