#pragma once
#include "servo_angles.h"

namespace calibrated {
constexpr Servo servos[] = {
    {"FR_pitch", 0, true, 970, 1500, 1890, 550},
    {"FL_pitch", 1, true, 900, 1420, 1800, 550},
    {"FR_yaw", 2, false, 1050, 1500, 1980, 0},
    {"FL_yaw", 3, false, 970, 1500, 1950, 0},
    {"RR_pitch", 8, true, 1000, 1500, 1900, 550},
    {"RL_pitch", 9, true, 970, 1500, 1880, 550},
    {"RR_yaw", 10, false, 950, 1500, 1850, 0},
    {"RL_yaw", 11, false, 1080, 1500, 1910, 0},
};
}
