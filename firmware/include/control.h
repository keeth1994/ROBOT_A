#pragma once
#include <array>
#include <cmath>
#include <cstdint>

namespace b91 {
constexpr double pi = 3.14159265358979323846;
constexpr double jointLimit = pi / 4;
using Targets = std::array<double, 8>;
inline double clamp(double x, double lo, double hi) { return x < lo ? lo : x > hi ? hi : x; }
inline double wrap(double x) { return std::atan2(std::sin(x), std::cos(x)); }
inline bool validRotation(const double r[3][3]) {
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) {
        double dot = 0;
        for (int k = 0; k < 3; ++k) dot += r[i][k] * r[j][k];
        if (!std::isfinite(dot) || std::abs(dot - (i == j ? 1 : 0)) > 1e-6) return false;
    }
    const double det = r[0][0]*(r[1][1]*r[2][2]-r[1][2]*r[2][1])
        - r[0][1]*(r[1][0]*r[2][2]-r[1][2]*r[2][0])
        + r[0][2]*(r[1][0]*r[2][1]-r[1][1]*r[2][0]);
    return std::abs(det - 1) < 1e-6;
}

// Exact course-profile waveform from robot_b91/gait.py. No simulator inputs.
inline Targets gait(double elapsed, double headingError, double lateralError) {
    Targets out{};
    constexpr double offsets[4] = {0, .5, .5, 0};
    const double ramp = clamp(elapsed, 0, 1);
    const double turn = clamp(3 * headingError, -.5, .5);
    const double side = clamp(10 * lateralError, -.5, .5);
    for (int i = 0; i < 4; ++i) {
        double yaw = 0, lift = 0;
        if (elapsed > 0) {
            double u = elapsed + offsets[i];
            u -= std::floor(u);
            if (u < .6) yaw = .45 * (2 * u / .6 - 1);
            else {
                const double v = (u - .6) / .4;
                yaw = .45 * (1 - 2 * v * v * (3 - 2 * v));
                lift = -.65 * std::pow(std::sin(pi * v), 2);
            }
        }
        const int sign = i < 2 ? 1 : -1;
        const int fore = (i == 0 || i == 2) ? 1 : -1;
        out[2 * i] = clamp(yaw * ramp * (sign + turn + fore * side), -jointLimit, jointLimit);
        out[2 * i + 1] = clamp(-.3 + lift * ramp, -jointLimit, jointLimit);
    }
    return out;
}

// Measure PWM at actual OUTPUT-joint angles; this includes yaw gearing and signs.
struct ServoCalibration { int channel; int minus45us; int zeroUs; int plus45us; };
inline bool valid(const std::array<ServoCalibration, 8>& servos) {
    uint16_t used = 0;
    for (const auto& s : servos) {
        if (s.channel < 0 || s.channel > 15 || (used & (1u << s.channel))) return false;
        used |= 1u << s.channel;
        for (int pulse : {s.minus45us, s.zeroUs, s.plus45us})
            if (pulse < 500 || pulse > 2500) return false;
        const int a = s.zeroUs - s.minus45us, b = s.plus45us - s.zeroUs;
        if (std::abs(a) < 20 || std::abs(b) < 20 || (a > 0) != (b > 0)) return false;
    }
    return true;
}
inline int pulse(const ServoCalibration& s, double angle) {
    if (!std::isfinite(angle)) return -1;
    const double fraction = clamp(angle / jointLimit, -1, 1);
    return static_cast<int>(std::lround(s.zeroUs + (fraction < 0
        ? -fraction * (s.minus45us - s.zeroUs) : fraction * (s.plus45us - s.zeroUs))));
}

// Wrap-safe watchdog. Expiry pauses; a new DRIVE alone cannot restart walking.
class Motion {
public:
    bool armed = false, walking = false, hasDrive = false;
    double turn = 0, side = 0, elapsed = 0, headingTarget = 0;
    uint32_t lastDrive = 0;
    void arm(double yaw, uint32_t now) {
        armed = true; walking = false; hasDrive = false; elapsed = 0; headingTarget = yaw;
        turn = side = 0; lastDrive = now;
    }
    bool drive(double t, double s, uint32_t now) {
        if (!armed || !std::isfinite(t) || !std::isfinite(s) || std::abs(t) > 1 || std::abs(s) > 1)
            return false;
        turn = t; side = s; lastDrive = now; hasDrive = true; return true;
    }
    bool walk(uint32_t now) {
        if (!armed || !hasDrive || uint32_t(now - lastDrive) > 300) return false;
        walking = true; return true;
    }
    void pause() { walking = false; turn = side = 0; }
    void disarm() { pause(); armed = false; }
    bool tick(uint32_t now, double dt) {
        if (uint32_t(now - lastDrive) > 500 || dt > .1 || dt <= 0) pause();
        if (!walking) return false;
        elapsed += dt;
        headingTarget = wrap(headingTarget + turn * .5 * dt);
        return true;
    }
};
} // namespace b91
