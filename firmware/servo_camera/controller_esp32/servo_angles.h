#pragma once
#include <cmath>

namespace calibrated {
struct Servo {
  const char *name;
  int channel;
  bool pitch;
  int minus45us, zeroUs, plus45us, minus90us;
};
inline int minAngle(const Servo &s) { return s.pitch ? -90 : -45; }
inline int restAngle(const Servo &s) { return s.pitch ? 45 : 0; }
inline bool valid(const Servo &s) {
  if (s.channel < 0 || s.channel > 15) return false;
  const int points[] = {s.minus90us, s.minus45us, s.zeroUs, s.plus45us};
  const int first = s.pitch ? 0 : 1;
  const bool increasing = s.zeroUs > s.minus45us;
  for (int i = first; i < 4; ++i) {
    if (points[i] < 500 || points[i] > 2500) return false;
    if (i > first) {
      const int difference = points[i] - points[i - 1];
      if (std::abs(difference) < 20 || (difference > 0) != increasing) return false;
    }
  }
  return true;
}

inline int pulse(const Servo &s, double angle) {
  if (!valid(s) || !std::isfinite(angle) || angle < minAngle(s) || angle > 45) return -1;
  const int points[] = {s.minus90us, s.minus45us, s.zeroUs, s.plus45us};
  const double position = (angle + 90) / 45;
  const int segment = position >= 2 ? 2 : static_cast<int>(position);
  return static_cast<int>(std::lround(points[segment] +
      (position - segment) * (points[segment + 1] - points[segment])));
}

inline double sweepAngle(const Servo &s, unsigned phase, double progress) {
  const double rest = restAngle(s);
  const double from = phase == 1 ? rest : (phase == 2 ? -45 : 45);
  const double to = phase == 1 ? -45 : (phase == 2 ? 45 : rest);
  return from + (to - from) * progress;
}
}
