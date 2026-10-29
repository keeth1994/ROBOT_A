// Measured output-joint angles: -45, 0, +45 degrees.
constexpr std::array<b91::ServoCalibration, 8> servos{{
    {8, 1220, 1500, 1790}, // FL_yaw
    {3, 950, 1500, 1850}, // FL_pitch
    {1, 1190, 1490, 1840}, // RL_yaw
    {2, 940, 1460, 1830}, // RL_pitch
    {0, 1120, 1415, 1705}, // FR_yaw
    {11, 1050, 1540, 1950}, // FR_pitch
    {9, 1180, 1500, 1800}, // RR_yaw
    {10, 960, 1480, 1830}, // RR_pitch
}};
