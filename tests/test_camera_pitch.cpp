// Run with g++ -std=c++11 tests/test_camera_pitch.cpp -o /tmp/camera-pitch-check.
#include <cassert>
#include <limits>

#include "../firmware/servo_camera/controller_esp32/camera_pitch.h"

int main() {
  assert(camera_pitch::pulse(-45) == 970);
  assert(camera_pitch::pulse(-65) == 783);
  assert(camera_pitch::pulse(-70) == 737);
  assert(camera_pitch::pulse(-80) == 643);
  assert(camera_pitch::pulse(-4) == 1453);
  assert(camera_pitch::pulse(0) == 1500);
  assert(camera_pitch::pulse(45) == 1890);
  assert(camera_pitch::pulse(55) == -1);
  assert(camera_pitch::pulse(56) == -1);
  assert(camera_pitch::pulse(-81) == -1);
  assert(camera_pitch::pulse(std::numeric_limits<double>::quiet_NaN()) == -1);
  // The camera correction must not extend the positive leg servo limit.
  assert(calibrated::pulse(camera_pitch::servo, 55) == -1);
}
