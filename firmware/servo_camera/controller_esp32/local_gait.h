#pragma once
#include <cmath>
#include <cstdint>
#include <cstring>

namespace walking {
constexpr double pi = 3.14159265358979323846;
inline double clamp(double x, double low, double high) {
  return x < low ? low : x > high ? high : x;
}

struct Parameters {
  double period = 1, duty = .6, yawAmplitude = .45, pitchLift = .65;
  double pitchStance = 0, pitchMin = -pi / 4;
  double headingGain = .8, lateralGain = 5, strafeGain = 0;
  bool crawl = false;
};

// Same joint order and radians as robot_b91/gait.py: FL, RL, FR, RR.
inline void targets(const Parameters &p, double elapsed, double gaitTime,
                    double heading, double lateral, double angles[8]) {
  const double trot[] = {0, .5, .5, 0}, crawl[] = {0, .5, .25, .75};
  double correction = clamp(p.headingGain * (heading + std::atan(p.lateralGain * lateral)), -.5, .5);
  double strafe = clamp(p.strafeGain * lateral, -.5, .5);
  for (int i = 0; i < 4; ++i) {
    double yaw = 0, lift = 0;
    if (elapsed > 0) {
      double u = std::fmod(gaitTime / p.period + (p.crawl ? crawl[i] : trot[i]), 1.0);
      if (u < p.duty) yaw = p.yawAmplitude * (2 * u / p.duty - 1);
      else {
        double v = (u - p.duty) / (1 - p.duty);
        yaw = p.yawAmplitude * (1 - 2 * v * v * (3 - 2 * v));
        lift = -p.pitchLift * std::pow(std::sin(pi * v), 2);
      }
      double ramp = clamp(elapsed / p.period, 0, 1);
      lift *= ramp;
      yaw *= ramp *
             ((i < 2 ? 1 : -1) + correction + (i % 2 == 0 ? 1 : -1) * strafe);
    }
    angles[2 * i] = clamp(yaw, -pi / 4, pi / 4);
    angles[2 * i + 1] = clamp(p.pitchStance + lift, p.pitchMin, 0);
  }
}

inline int jointIndex(const char *name) {
  const char *names[] = {"FL_yaw", "FL_pitch", "RL_yaw", "RL_pitch",
                         "FR_yaw", "FR_pitch", "RR_yaw", "RR_pitch"};
  for (int i = 0; i < 8; ++i) if (std::strcmp(name, names[i]) == 0) return i;
  return -1;
}

inline double servoAngle(int joint, double radians) {
  // Rear yaw mounting is reversed; physical pitch +45 is foot-down.
  if (joint == 2 || joint == 6) radians = -radians;
  return radians * 180 / pi + (joint % 2 ? 45 : 0);
}

struct Gait {
  Parameters params;
  bool useImu = false, ready = false, stopping = false;
  uint32_t started = 0, lastUpdate = 0, stopStarted = 0, settleMs = 0;
  double elapsed = 0, gaitTime = 0, speed = 1;
  double headingOffset = 0, lateral = 0;
  double angles[8] = {}, stopAngles[8] = {};

  void begin(uint32_t now, const Parameters &p, bool imu, uint32_t settling) {
    *this = Gait();
    params = p; useImu = imu; settleMs = settling;
    started = lastUpdate = now;
    targets(params, 0, 0, 0, 0, angles);
  }

  void stop(uint32_t now) {
    if (stopping) return;
    stopping = true; stopStarted = now;
    for (int i = 0; i < 8; ++i) stopAngles[i] = angles[i];
  }

  bool stopped(uint32_t now) const { return stopping && uint32_t(now - stopStarted) >= 300; }

  void update(uint32_t now, bool imuFresh, double heading) {
    double dt = uint32_t(now - lastUpdate) / 1000.0;
    lastUpdate = now;
    if (stopping) {
      double u = clamp(uint32_t(now - stopStarted) / 300.0, 0, 1);
      u = u * u * (3 - 2 * u);
      for (int i = 0; i < 8; ++i)
        angles[i] = stopAngles[i] + u * ((i % 2 ? params.pitchStance : 0) - stopAngles[i]);
      return;
    }
    // Pause phase on missing observations or a stalled loop; never catch up in one jump.
    if (!ready || (useImu && !imuFresh) || uint32_t(now - started) < settleMs || dt > .1) return;
    elapsed += dt;
    gaitTime += dt * speed;
    targets(params, elapsed, gaitTime, headingOffset + (useImu ? heading : 0), lateral, angles);
  }
};
}
