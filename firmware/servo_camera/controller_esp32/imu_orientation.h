#pragma once
#include "local_gait.h"

// Body axes: X forward, Y left, Z up. Inputs are g and degrees/second.
// Accelerometer correction estimates tilt; yaw remains relative and can drift.
struct ImuOrientation {
  double roll = 0, pitch = 0, heading = 0;
  double bias[3] = {}, sum[3] = {};
  unsigned samples = 0;
  uint32_t lastSample = 0, started = 0;
  bool initialized = false, sampled = false;

  void reset(uint32_t now) { *this = ImuOrientation(); started = now; }

  void update(uint32_t now, double ax, double ay, double az, double gx, double gy, double gz) {
    double dt = sampled ? uint32_t(now - lastSample) / 1000.0 : 0;
    if (dt > .1) dt = 0;
    sampled = true; lastSample = now;
    gx *= walking::pi / 180; gy *= walking::pi / 180; gz *= walking::pi / 180;
    uint32_t elapsed = now - started;
    if (elapsed > 1000 && elapsed < 1900 && std::sqrt(gx * gx + gy * gy + gz * gz) < .05) {
      const double gyro[] = {gx, gy, gz};
      ++samples;
      for (int i = 0; i < 3; ++i) { sum[i] += gyro[i]; bias[i] = sum[i] / samples; }
    }
    gx -= bias[0]; gy -= bias[1]; gz -= bias[2];
    double sr = std::sin(roll), cr = std::cos(roll), cp = walking::clamp(std::cos(pitch), .3, 1);
    // Start heading at the end of settling; keep early gyro bias out of the reference.
    if (elapsed > 2000) {
      double headingDt = std::fmin(dt, (elapsed - 2000) / 1000.0);
      heading += (gy * sr + gz * cr) / cp * headingDt;
    }
    roll += (gx + std::tan(pitch) * (gy * sr + gz * cr)) * dt;
    pitch += (gy * cr - gz * sr) * dt;
    double acceleration = std::sqrt(ax * ax + ay * ay + az * az) * 9.80665;
    if (acceleration > 8 && acceleration < 11.5) {
      double gain = initialized ? 1 - std::exp(-dt / 1.5) : 1;
      roll += gain * (std::atan2(ay, az) - roll);
      pitch += gain * (std::atan2(-ax, std::hypot(ay, az)) - pitch);
      initialized = true;
    }
  }
};
