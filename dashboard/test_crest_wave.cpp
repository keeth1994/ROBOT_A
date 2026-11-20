// Host harness for parity with the simulator and firmware pause/stop behavior.
#include "../firmware/servo_camera/controller_esp32/local_gait.h"
#include <cassert>
#include <iomanip>
#include <iostream>
#include <string>

int main() {
  walking::Parameters p;
  p.crest = true; p.duty = .8; p.pitchStance = 0; p.yawBias = -.25;
  walking::Gait gait;
  gait.begin(1000, p, true, 2000);
  gait.ready = true;
  gait.update(1020, true, 0);
  assert(gait.gaitTime == 0); // Settling.
  gait.lastUpdate = 3000;
  gait.update(3020, true, 0);
  const double phase = gait.gaitTime;
  double before[8];
  for (int i = 0; i < 8; ++i) before[i] = gait.angles[i];
  gait.update(3040, false, 0); // Missing IMU freezes phase and targets.
  assert(gait.gaitTime == phase);
  for (int i = 0; i < 8; ++i) assert(gait.angles[i] == before[i]);
  gait.ready = false;
  gait.update(3060, true, 0);
  assert(gait.gaitTime == phase);
  gait.ready = true;
  gait.update(3080, true, 0);
  assert(std::abs(gait.gaitTime - phase - .02) < 1e-10);
  gait.stop(3080);
  gait.update(3380, true, 0);
  assert(gait.stopped(3380));
  for (int leg = 0; leg < 4; ++leg) assert(gait.angles[2*leg+1] == 0);

  // Exactly one lateral cycle, even if speed changes, observations pause or
  // polling would overshoot the boundary. Completed feet never sweep home.
  p.sideways = true; p.period = 2; p.yawBias = 0;
  gait.begin(0, p, true, 2000);
  gait.sideways = 1; gait.ready = true; gait.speed = .65;
  for (uint32_t now = 20; now < 7000 && !gait.stopping; now += 20) {
    bool fresh = now < 2600 || now >= 3000;
    if (now > 4000) gait.speed = .5;
    gait.update(now, fresh, 0);
  }
  assert(gait.sidestepComplete && gait.stopping);
  assert(gait.gaitTime == p.period);
  for (int i = 0; i < 8; ++i) before[i] = gait.angles[i];
  for (int leg = 0; leg < 4; ++leg) assert(gait.angles[2*leg+1] == 0);
  gait.update(7500, true, 0);
  assert(gait.stopped(7500));
  for (int i = 0; i < 8; ++i) assert(gait.angles[i] == before[i]);
  gait.begin(8000, p, true, 2000);
  assert(!gait.sidestepComplete && !gait.stopping && gait.gaitTime == 0);

  std::cout << std::setprecision(17);
  std::string pattern;
  double elapsed, time, heading, lateral, sideways;
  while (std::cin >> pattern >> p.period >> p.duty >> p.yawAmplitude >> p.pitchLift
         >> p.pitchStance >> p.yawBias >> p.pitchMin >> p.headingGain
         >> p.lateralGain >> p.strafeGain >> elapsed >> time >> heading >> lateral >> p.sideways >> sideways >> p.turnLimit >> p.rearStrideScale) {
    p.crest = pattern == "crest"; p.crawl = pattern == "crawl";
    p.steeringTrim = 0; // Python caller accounts for trim separately.
    double target[8];
    walking::targets(p, elapsed, time, heading, lateral, target, 1, sideways);
    for (double angle : target) std::cout << angle << ' ';
    std::cout << '\n';
  }
}
