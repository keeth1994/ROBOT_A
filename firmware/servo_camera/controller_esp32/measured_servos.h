#pragma once
#include "servo_angles.h"

namespace calibrated {
constexpr Servo servos[] = {
    {"FR_pitch", 0, true, 970, 1500, 1860, 550},
    {"FL_pitch", 1, true, 900, 1420, 1800, 550},
    {"FR_yaw", 2, false, 1050, 1500, 1980, 0},
    {"FL_yaw", 3, false, 975, 1450, 1950, 0},
    {"RR_pitch", 8, true, 1000, 1500, 1910, 550},
    {"RL_pitch", 9, true, 970, 1500, 1860, 550},
    {"RR_yaw", 10, false, 920, 1500, 1850, 0},
    {"RL_yaw", 11, false, 1050, 1550, 1950, 0},
};
}
