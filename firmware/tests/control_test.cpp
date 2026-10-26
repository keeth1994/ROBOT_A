#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <iostream>
#include <limits>
#include "control.h"

int main(int argc, char**) {
    if (argc > 1) {
        double t, heading, lateral;
        std::cout.precision(17);
        while (std::cin >> t >> heading >> lateral) {
            for (double angle : b91::gait(t, heading, lateral)) std::cout << angle << ' ';
            std::cout << '\n';
        }
        return 0;
    }
    std::array<b91::ServoCalibration, 8> s{};
    double rotation[3][3] = {{1,0,0}, {0,1,0}, {0,0,1}};
    assert(b91::validRotation(rotation));
    rotation[0][0] = -1; assert(!b91::validRotation(rotation));
    assert(!b91::valid(s));
    for (int i = 0; i < 8; ++i) s[i] = {i, 1000, 1500, 2000};
    assert(b91::valid(s));
    assert(b91::pulse(s[0], -99) == 1000 && b91::pulse(s[0], 99) == 2000);
    assert(b91::pulse(s[0], 0) == 1500);
    assert(b91::pulse(s[0], std::numeric_limits<double>::quiet_NaN()) == -1);
    s[0] = {0, 2000, 1400, 900};
    assert(b91::valid(s));
    assert(b91::pulse(s[0], -b91::jointLimit/2) == 1700);
    assert(b91::pulse(s[0], b91::jointLimit/2) == 1150);
    s[0].channel = 1; assert(!b91::valid(s)); s[0].channel = 0;
    s[0].zeroUs = 2100; assert(!b91::valid(s));
    s[0] = {-1, 1000, 1500, 2000}; assert(!b91::valid(s));

    b91::Motion m;
    assert(!m.walk(0) && !m.drive(0,0,0));
    m.arm(0, 0); assert(!m.walk(0));
    assert(!m.drive(std::numeric_limits<double>::infinity(), 0, 0));
    assert(!m.drive(1.1, 0, 0));
    assert(m.drive(.2, -.3, 0) && m.walk(0));
    assert(m.tick(20, .02));
    assert(m.elapsed == .02 && m.headingTarget > 0);
    assert(!m.tick(501, .02) && !m.walking);
    const double elapsed = m.elapsed;
    assert(m.drive(0,0,502)); assert(!m.tick(522,.02)); // no automatic restart
    assert(m.walk(522) && m.tick(542,.02) && m.elapsed > elapsed);
    m.pause(); assert(!m.tick(562,.02));
    m.disarm(); assert(!m.walk(562));
    m.arm(0, UINT32_MAX-10); assert(m.drive(0,0,UINT32_MAX-10));
    assert(m.walk(UINT32_MAX-5) && m.tick(10,.02)); // millis rollover
    assert(!m.tick(20,.11)); // overrun stops motion
    std::cout << "PASS: pulse calibration, travel bounds, watchdog, restart and timer rollover\n";
}
