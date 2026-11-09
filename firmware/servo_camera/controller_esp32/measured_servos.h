#pragma once
#include "servo_angles.h"

namespace calibrated {
constexpr Servo servos[] = {
    {"FR_pitch", 0, true, 1050, 1540, 1950, 550},
    {"FL_pitch", 1, true, 950, 1500, 1850, 521},
    {"FR_yaw", 2, false, 1120, 1415, 1705, 0},
    {"FL_yaw", 3, false, 1220, 1500, 1790, 0},
    {"RR_pitch", 8, true, 960, 1480, 1830, 550},
    {"RL_pitch", 9, true, 940, 1460, 1830, 516},
    {"RR_yaw", 10, false, 1180, 1500, 1800, 0},
    {"RL_yaw", 11, false, 1190, 1490, 1840, 0},
};
}
