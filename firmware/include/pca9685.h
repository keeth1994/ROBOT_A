#pragma once
#include <Wire.h>
#include "hardware_config.h"

// PCA9685 register protocol (NXP datasheet). Every write checks I2C acknowledgement.
class ServoBus {
    uint8_t prescale = 121;
    bool write(uint8_t reg, const uint8_t* bytes, size_t size) {
        Wire.beginTransmission(hw::servoAddress);
        Wire.write(reg); Wire.write(bytes, size);
        return Wire.endTransmission() == 0;
    }
    bool reg(uint8_t address, uint8_t value) { return write(address, &value, 1); }
public:
    bool off() { const uint8_t bytes[] = {0, 0, 0, 0x10}; return write(0xFA, bytes, 4); }
    bool begin() {
        prescale = uint8_t(std::lround(hw::pwmOscillatorHz / (4096 * 50)) - 1);
        // Auto-increment; sleep before changing prescale; disable broadcast ALLCALL.
        if (!reg(0x00, 0x30) || !off() || !reg(0x01, 0x04) || !reg(0xFE, prescale)) return false;
        if (!reg(0x00, 0x20)) return false;
        delay(1);
        return reg(0x00, 0xA0) && off();
    }
    bool set(int channel, int microseconds) {
        if (channel < 0 || channel > 15 || microseconds < 500 || microseconds > 2500) return false;
        const auto ticks = uint16_t(std::lround(microseconds * hw::pwmOscillatorHz / (1e6 * (prescale + 1))));
        const uint8_t bytes[] = {0, 0, uint8_t(ticks & 255), uint8_t(ticks >> 8)};
        return write(uint8_t(0x06 + channel * 4), bytes, 4);
    }
};
