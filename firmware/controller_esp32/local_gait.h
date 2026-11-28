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
  double frontPitchLift = -1; // Negative uses the shared foot lift.
  // +45° is foot down; a 125° lift reaches -80°.
  double pitchStance = 0, pitchMin = -125 * pi / 180;
  double headingGain = .8, lateralGain = 5, strafeGain = 0;
  double yawBias = 0; // Stance offset. Yaw stays within ±45°.
  double rearStrideScale = 1; // Scale the rear stride without changing foot height.
  bool crawl = false, crest = false, corridor = false, search = false;
  double steeringTrim = 0.01;
};

// Angles in radians: FL, RL, FR, RR; yaw before pitch.
inline void targets(const Parameters &p, double elapsed, double gaitTime,
                    double heading, double lateral, double angles[8], double forwardScale = 1) {
  const double trot[] = {0, .5, .5, 0}, crawl[] = {0, .5, .25, .75};
  const double crest[] = {0, .25, .5, .75};
  double turnLimit = p.corridor ? .95 : .5;
  double correction = clamp(
    p.headingGain * (heading + std::atan(p.lateralGain * lateral))
    + p.steeringTrim,
    -turnLimit,
    turnLimit
  );
  double forward = p.corridor ? std::fmax(.25, 1 - std::abs(correction)) : 1;
  // Allow turning without moving forward.
  if (p.search) forward = clamp(forwardScale, 0, 1);
  // Limit sideways movement.
  double strafe = clamp(p.strafeGain * lateral, -.3, .3);
  for (int i = 0; i < 4; ++i) {
    double yaw = 0, lift = 0;
    if (elapsed > 0) {
      double u = std::fmod(gaitTime / p.period + (p.crest ? crest[i] : p.crawl ? crawl[i] : trot[i]), 1.0);
      if (u < p.duty) yaw = p.yawAmplitude * (2 * u / p.duty - 1);
      else {
        double v = (u - p.duty) / (1 - p.duty);
        // Lift one foot at a time during the crest gait.

        double sweep = p.crest ? clamp((v - .2) / .6, 0, 1) : v;
        yaw = p.yawAmplitude * (1 - 2 * sweep * sweep * (3 - 2 * sweep));
        // Lift before swinging forward.
        double height = clamp((v < .5 ? v : 1 - v) / (p.crest ? .2 : .15), 0, 1);
        double pitchLift = p.crest && i % 2 == 0 && p.frontPitchLift >= 0 ? p.frontPitchLift : p.pitchLift;
        lift = -pitchLift * height * height * (3 - 2 * height);
      }
      double ramp = clamp(elapsed / p.period, 0, 1);
      lift *= ramp;
      yaw *= ramp * (i % 2 ? p.rearStrideScale : 1) *
             ((i < 2 ? forward : -forward) + correction + (i % 2 == 0 ? 1 : -1) * strafe);
    }
    double bias = clamp(elapsed / p.period, 0, 1) * p.yawBias *
                  ((i == 0 || i == 3) ? -1 : 1);
    angles[2 * i] = clamp(clamp(yaw, -pi / 4, pi / 4) + bias, -pi / 4, pi / 4);
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
  // Reverse rear yaw. Pitch +45° means foot down.
  if (joint == 2 || joint == 6) radians = -radians;
  return radians * 180 / pi + (joint % 2 ? 45 : 0);
}

constexpr double danceIntroSeconds = 10, danceYawStepSeconds = .5;
constexpr int danceYawRepeats = 3;
constexpr double danceYawSeconds = 2 + 8 * danceYawRepeats * danceYawStepSeconds;
constexpr double danceCycleSeconds = 14 + danceYawSeconds;

inline void diagonalDanceTargets(double danceTime, double angles[8]) {
  for (int leg = 0; leg < 4; ++leg) {
    double elapsed = danceTime - ((leg == 0 || leg == 3) ? 0 : .5);
    double lift = 0;
    if (elapsed >= 0) {
      double step = std::fmod(elapsed, 1.0);
      // Start the next pair before this pair finishes lowering.
      lift = step < .2 ? step / .2 : step < .72 ? (.72 - step) / .52 : 0;
      lift = clamp(lift, 0, 1);
      lift = lift * lift * (3 - 2 * lift);
    }
    angles[2 * leg] = 0;
    angles[2 * leg + 1] = -65 * pi / 180 * lift;
  }
}

inline void lyingYawTargets(double elapsed, double angles[8]) {
  const double prepareEnd = 1, sweepSeconds = danceYawStepSeconds;
  const double sweepEnd = prepareEnd + 8 * danceYawRepeats * sweepSeconds;
  const int forwardOrder[] = {0, 2, 3, 1}, reverseOrder[] = {0, 1, 3, 2};
  int step = elapsed >= prepareEnd && elapsed < sweepEnd ?
             static_cast<int>((elapsed - prepareEnd) / sweepSeconds) : -1;
  bool reverse = step >= 0 && step % 8 >= 4;
  const int *order = reverse ? reverseOrder : forwardOrder;
  for (int position = 0; position < 4; ++position) {
    int leg = order[position];
    double yaw, pitch = -80 * pi / 180;
    if (elapsed < prepareEnd) {
      double u = clamp(elapsed / prepareEnd, 0, 1);
      yaw = pi / 4 * u * u * (3 - 2 * u);
    } else if (elapsed < sweepEnd) {
      int roundStep = step % 8;
      int active = roundStep % 4;
      double u = (elapsed - prepareEnd) / sweepSeconds - step;
      // Add a foot lift during the last two repeats.
      if (step >= 8 && position == active)
        pitch -= 45 * pi / 180 * (1 - std::cos(2 * pi * u)) / 2;
      u = u * u * (3 - 2 * u);
      double direction = reverse ? (position < active ? 1 : -1) :
                                   (position < active ? -1 : 1);
      if (position == active) direction = reverse ? 2 * u - 1 : 1 - 2 * u;
      yaw = pi / 4 * direction;
    } else {
      double u = clamp(elapsed - sweepEnd, 0, 1);
      yaw = pi / 4 * (1 - u * u * (3 - 2 * u));
    }
    yaw = clamp(yaw, -pi / 4, pi / 4);
    angles[2 * leg] = yaw;
    angles[2 * leg + 1] = pitch;
  }
}

inline void danceTargets(double danceTime, double angles[8]) {
  const double poseEnd = 1, raiseEnd = poseEnd + .5, holdEnd = raiseEnd + 2;
  const double waveEnd = holdEnd + 5, lowerEnd = waveEnd + .5, introEnd = danceIntroSeconds;
  if (danceTime < introEnd) {
    for (int joint = 0; joint < 8; ++joint) angles[joint] = 0;
    double stance = clamp(danceTime < poseEnd ? danceTime / poseEnd :
                          danceTime < lowerEnd ? 1 : (introEnd - danceTime), 0, 1);
    stance = stance * stance * (3 - 2 * stance);
    angles[2] = angles[4] = pi / 4 * stance;
    angles[6] = -pi / 4 * stance;
    double lift = danceTime < raiseEnd ? (danceTime - poseEnd) / .5 :
                  danceTime < waveEnd ? 1 : (lowerEnd - danceTime) / .5;
    lift = clamp(lift, 0, 1);
    angles[1] = -125 * pi / 180 * lift * lift * (3 - 2 * lift);
    double point = clamp((danceTime - poseEnd) / .5, 0, 1);
    point = point * point * (3 - 2 * point);
    double returnYaw = clamp((danceTime - holdEnd) / .5, 0, 1);
    returnYaw = returnYaw * returnYaw * (3 - 2 * returnYaw);
    angles[0] = -40 * pi / 180 * point * (1 - returnYaw);
    if (danceTime >= holdEnd && danceTime < waveEnd) {
      double waveTime = danceTime - holdEnd;
      angles[1] += 35 * pi / 180 * (1 - std::cos(2 * pi * 1.6 * waveTime)) / 2;
    }
    return;
  }
  const double diagonalEnd = 5, lieDownEnd = diagonalEnd + 2;
  const double legWaveEnd = lieDownEnd + 5, yawEnd = legWaveEnd + danceYawSeconds;
  const double cycleEnd = yawEnd + 2;
  double cycleTime = std::fmod(danceTime - introEnd, cycleEnd);
  double diagonalTime = std::fmin(cycleTime, diagonalEnd);
  diagonalDanceTargets(diagonalTime, angles);
  if (cycleTime < diagonalEnd) return;
  if (cycleTime >= legWaveEnd && cycleTime < yawEnd) {
    lyingYawTargets(cycleTime - legWaveEnd, angles);
    return;
  }

  const double lyingPitch = -80 * pi / 180;
  if (cycleTime < lieDownEnd) {
    double u = (cycleTime - diagonalEnd) / 2;
    u = u * u * (3 - 2 * u);
    for (int leg = 0; leg < 4; ++leg)
      angles[2 * leg + 1] += u * (lyingPitch - angles[2 * leg + 1]);
  } else {
    double pitch = lyingPitch, yaw = 0;
    if (cycleTime < legWaveEnd) {
      double waveTime = cycleTime - lieDownEnd;
      pitch -= 45 * pi / 180 * (1 - std::cos(2 * pi * waveTime)) / 2;
      double ramp = clamp(std::fmin(waveTime, 5 - waveTime) / .5, 0, 1);
      ramp = ramp * ramp * (3 - 2 * ramp);
      yaw = 20 * pi / 180 * std::sin(2 * pi * waveTime) * ramp;
    } else {
      double u = (cycleTime - yawEnd) / 2;
      pitch *= 1 - u * u * (3 - 2 * u);
    }
    for (int leg = 0; leg < 4; ++leg) {
      angles[2 * leg] = (leg < 2 ? 1 : -1) * yaw;
      angles[2 * leg + 1] = pitch;
    }
  }
}

struct Gait {
  Parameters params;
  bool useImu = false, dance = false, ready = false, stopping = false;
  uint32_t started = 0, lastUpdate = 0, stopStarted = 0, settleMs = 0;
  double elapsed = 0, gaitTime = 0, speed = 1;
  double headingOffset = 0, lateral = 0, forward = 1;
  double angles[8] = {}, stopAngles[8] = {};
  int danceTurnPhase = 0;
  bool danceTurnDone = false;
  double danceTurnTime = 0, danceTurnHeading = 0;
  uint32_t danceTurnStarted = 0;
  double danceTurnAngles[8] = {};
  const char *danceError = nullptr;

  void begin(uint32_t now, const Parameters &p, bool imu, uint32_t settling, bool dancing = false) {
    *this = Gait();
    params = p; useImu = imu; settleMs = settling; dance = dancing;
    started = lastUpdate = now;
    if (dance) danceTargets(0, angles);
    else targets(params, 0, 0, 0, 0, angles);
  }

  void stop(uint32_t now) {
    if (stopping) return;
    stopping = true; stopStarted = now;
    for (int i = 0; i < 8; ++i) stopAngles[i] = angles[i];
  }

  bool stopped(uint32_t now) const { return stopping && uint32_t(now - stopStarted) >= 300; }

  void updateDanceTurn(uint32_t now, double dt, double heading) {
    if (danceTurnPhase == 1) {
      double turned = std::abs(heading - danceTurnHeading);
      if (turned >= 2 * pi) {
        danceTurnPhase = 2;
        danceTurnTime = 0;
        for (int i = 0; i < 8; ++i) danceTurnAngles[i] = angles[i];
      } else if (uint32_t(now - danceTurnStarted) >= 45000) {
        danceError = "Dance turn timed out. Check foot grip and yaw movement; restart explicitly.";
        stop(now);
        return;
      } else {
        danceTurnTime += dt * speed;
        diagonalDanceTargets(danceTurnTime, angles);
        double ramp = clamp(danceTurnTime, 0, 1);
        ramp = ramp * ramp * (3 - 2 * ramp);
        double amplitude = 20 * pi / 180 * ramp * clamp((2 * pi - turned) / (pi / 4), .2, 1);
        for (int leg = 0; leg < 4; ++leg) {
          double phase = std::fmod(danceTurnTime + ((leg == 0 || leg == 3) ? 0 : .5), 1.0);
          double u = phase < .72 ? phase / .72 : (phase - .72) / .28;
          u = u * u * (3 - 2 * u);
          angles[2 * leg] = amplitude * (phase < .72 ? 2 * u - 1 : 1 - 2 * u);
        }
        return;
      }
    }
    danceTurnTime += dt * speed;
    double u = clamp(danceTurnTime, 0, 1);
    u = u * u * (3 - 2 * u);
    double target[8];
    danceTargets(gaitTime, target);
    for (int i = 0; i < 8; ++i)
      angles[i] = danceTurnAngles[i] + u * (target[i] - danceTurnAngles[i]);
    if (danceTurnTime >= 1) {
      danceTurnPhase = 0;
      danceTurnDone = true;
    }
  }

  void update(uint32_t now, bool imuFresh, double heading) {
    double dt = uint32_t(now - lastUpdate) / 1000.0;
    lastUpdate = now;
    if (stopping) {
      double u = clamp(uint32_t(now - stopStarted) / 300.0, 0, 1);
      u = u * u * (3 - 2 * u);
      for (int i = 0; i < 8; ++i) {
        double stance = i % 2 ? params.pitchStance :
                        params.yawBias * ((i == 0 || i == 6) ? -1 : 1);
        angles[i] = stopAngles[i] + u * (stance - stopAngles[i]);
      }
      return;
    }
    if (dance && uint32_t(now - started) >= settleMs &&
        (!imuFresh || !std::isfinite(heading))) {
      danceError = "Dance needs fresh IMU readings. Check the IMU; restart explicitly.";
      stop(now);
      return;
    }
    // Pause the gait when readings or updates are late.
    if (!ready || (useImu && !imuFresh) || uint32_t(now - started) < settleMs || dt > .1) return;
    elapsed += dt;
    if (dance && !danceTurnDone) {
      if (!danceTurnPhase) {
        danceTurnPhase = 1;
        danceTurnTime = 0;
        danceTurnHeading = heading;
        danceTurnStarted = now;
      }
      updateDanceTurn(now, dt, heading);
      return;
    }
    gaitTime += dt * speed;
    if (dance) danceTargets(gaitTime, angles);
    else targets(params, elapsed, gaitTime, headingOffset + (useImu ? heading : 0), lateral, angles, forward);
  }
};
}
