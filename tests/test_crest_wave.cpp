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
  assert(gait.gaitTime == 0);
  gait.lastUpdate = 3000;
  gait.update(3020, true, 0);
  const double phase = gait.gaitTime;
  double before[8];
  for (int i = 0; i < 8; ++i) before[i] = gait.angles[i];
  gait.update(3040, false, 0);
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

  std::cout << std::setprecision(17);
  std::string pattern;
  double elapsed, time, heading, lateral;
  while (std::cin >> pattern >> p.period >> p.duty >> p.yawAmplitude >> p.pitchLift
         >> p.pitchStance >> p.yawBias >> p.pitchMin >> p.headingGain
         >> p.lateralGain >> p.strafeGain >> elapsed >> time >> heading >> lateral >> p.rearStrideScale >> p.frontPitchLift) {
    const bool ramp = pattern == "ramp" || pattern == "ramp_crest";
    p.search = p.corridor = pattern == "align" || ramp;
    double forward = pattern == "align" ? 0 : 1;
    if (ramp && !(std::cin >> forward)) return 1;
    p.crest = pattern == "crest" || pattern == "align" || pattern == "ramp_crest"; p.crawl = pattern == "crawl";
    p.steeringTrim = 0;
    double target[8];
    walking::targets(p, elapsed, time, heading, lateral, target, forward);
    for (double angle : target) std::cout << angle << ' ';
    std::cout << '\n';
  }
}
