#pragma once
#include "servo_angles.h"

namespace calibrated {
constexpr Servo servos[] = {
    {"FR_yaw", 0, false, 1120, 1415, 1705, 0},
    {"RL_yaw", 1, false, 1190, 1490, 1840, 0},
    {"RL_pitch", 2, true, 940, 1460, 1830, 516},
    {"FL_pitch", 3, true, 950, 1500, 1850, 521},
    {"FL_yaw", 8, false, 1220, 1500, 1790, 0},
    {"RR_yaw", 9, false, 1180, 1500, 1800, 0},
    {"RR_pitch", 10, true, 960, 1480, 1830, 550},
    {"FR_pitch", 11, true, 1050, 1540, 1950, 550},
};
}
