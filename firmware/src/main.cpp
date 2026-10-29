#include <Arduino.h>
#include <SparkFun_BMI270_Arduino_Library.h>
#include "control.h"
#include "hardware_config.h"
#include "pca9685.h"

BMI270 imu;
ServoBus servos;
b91::Motion motion;
b91::Targets targets = b91::gait(0, 0, 0);
bool busReady = false, imuReady = false, zeroed = false, faulted = false, attitudeReady = false;
double roll = 0, pitch = 0, yaw = 0, bias[3] = {};
uint32_t lastImu = 0, lastControl = 0, lastBench = 0;
bool benchActive = false;

void fault(const char* reason) {
    motion.disarm(); benchActive = false; faulted = true;
    // Best effort: if the bus is broken, existing PWM may continue. No hardware e-stop is implied.
    servos.off();
    Serial.printf("FAULT %s; reset required\n", reason);
}

void axes(const double in[3], double out[3]) {
    for (int i = 0; i < 3; ++i) {
        out[i] = 0;
        for (int j = 0; j < 3; ++j) out[i] += hw::imuRotation[i][j] * in[j];
    }
}

bool readImu(double a[3], double g[3]) {
    uint8_t status = 0;
    if (imu.getStatus(&status) != BMI2_OK ||
        (status & (BMI2_DRDY_ACC | BMI2_DRDY_GYR)) != (BMI2_DRDY_ACC | BMI2_DRDY_GYR) ||
        imu.getSensorData() != BMI2_OK) return false;
    const double rawA[] = {imu.data.accelX, imu.data.accelY, imu.data.accelZ};
    const double rawG[] = {imu.data.gyroX, imu.data.gyroY, imu.data.gyroZ};
    axes(rawA, a); axes(rawG, g);
    for (int i = 0; i < 3; ++i) if (!std::isfinite(a[i]) || !std::isfinite(g[i])) return false;
    return true;
}

void updateAttitude(uint32_t now) {
    double a[3], g[3];
    if (!readImu(a, g)) return;
    const double dt = (now - lastImu) * .001;
    lastImu = now;
    const double ar = std::atan2(a[1], a[2]);
    const double ap = std::atan2(-a[0], std::hypot(a[1], a[2]));
    if (!attitudeReady) { roll = ar; pitch = ap; yaw = 0; attitudeReady = true; return; }
    if (dt > .1 || dt <= 0) return;
    // Euler rates include roll/pitch: gyro Z alone is not yaw rate on a ramp.
    for (int i = 0; i < 3; ++i) g[i] = (g[i] - bias[i]) * b91::pi / 180;
    const double sr = std::sin(roll), cr = std::cos(roll), cp = std::cos(pitch);
    if (std::abs(cp) < .3) { if (motion.armed) fault("attitude out of range"); return; }
    roll += (g[0] + std::tan(pitch) * (sr * g[1] + cr * g[2])) * dt;
    pitch += (cr * g[1] - sr * g[2]) * dt;
    yaw = b91::wrap(yaw + (sr * g[1] + cr * g[2]) / cp * dt);
    const double norm = std::sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    if (norm > .85 && norm < 1.15) {
        const double gain = dt / (.5 + dt);
        roll += gain * b91::wrap(ar - roll); pitch += gain * b91::wrap(ap - pitch);
    }
    const double tilt = std::acos(b91::clamp(std::cos(roll) * std::cos(pitch), -1, 1));
    if (motion.armed && tilt > b91::pi / 4) fault("tilt exceeds 45 degrees");
}

bool writeTargets() {
    for (int i = 0; i < 8; ++i)
        if (!servos.set(hw::servos[i].channel, b91::jointPulse(hw::servos[i], i, targets[i]))) return false;
    return true;
}

void zeroImu() {
    if (motion.armed || benchActive || !imuReady || faulted) { Serial.println("ERR ZERO requires disarmed healthy IMU"); return; }
    zeroed = false;
    double sums[3] = {}, squares[3] = {};
    unsigned samples = 0;
    uint32_t start = millis();
    Serial.println("ZERO keep robot stationary for 2 seconds");
    while (samples < 200 && millis() - start < 4000) {
        double a[3], g[3];
        if (readImu(a, g)) {
            double norm = std::sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
            if (norm < .85 || norm > 1.15) { Serial.println("ERR ZERO robot moved"); return; }
            for (int i = 0; i < 3; ++i) { sums[i] += g[i]; squares[i] += g[i]*g[i]; }
            ++samples;
        }
        delay(10);
    }
    if (samples != 200) { Serial.println("ERR ZERO missing samples"); return; }
    for (int i = 0; i < 3; ++i) {
        double mean = sums[i] / samples;
        if (std::abs(mean) > 3 || squares[i] / samples - mean*mean > .25) {
            Serial.println("ERR ZERO robot rotating"); return;
        }
    }
    for (int i = 0; i < 3; ++i) bias[i] = sums[i] / samples;
    zeroed = true; attitudeReady = false; lastImu = lastControl = millis();
    Serial.println("OK ZERO");
}

void command(const char* line) {
    float a = 0, b = 0; int channel = -1, width = 0; char extra;
    uint32_t now = millis();
    if (!strcmp(line, "STATUS")) {
        Serial.printf("STATUS calibrated=%d bus=%d imu=%d zeroed=%d armed=%d walking=%d fault=%d roll=%.3f pitch=%.3f yaw=%.3f\n",
            hw::calibrated && b91::valid(hw::servos), busReady, imuReady, zeroed,
            motion.armed, motion.walking, faulted, roll, pitch, yaw);
    } else if (!strcmp(line, "ZERO")) zeroImu();
    else if (!strcmp(line, "OFF")) {
        motion.disarm(); benchActive = false;
        if (servos.off()) Serial.println("OK OFF"); else fault("OFF I2C write failed");
    } else if (!strcmp(line, "STOP")) { motion.pause(); Serial.println("OK STOP holding current targets"); }
    else if (!strcmp(line, "ARM")) {
        if (motion.armed || benchActive || faulted || !busReady || !imuReady || !zeroed || !attitudeReady ||
            now - lastImu > 100 || !hw::calibrated || !b91::valid(hw::servos) || !b91::validRotation(hw::imuRotation) ||
            std::cos(roll)*std::cos(pitch) < std::cos(b91::pi/6)) {
            Serial.println("ERR ARM requires calibrated hardware, ZERO, upright fresh IMU and disarmed state"); return;
        }
        motion.arm(yaw, now); targets = b91::gait(0, 0, 0);
        if (!writeTargets()) fault("ARM I2C write failed"); else Serial.println("OK ARM");
    } else if (!strcmp(line, "WALK")) {
        if (motion.walk(now)) Serial.println("OK WALK"); else Serial.println("ERR WALK requires ARM and recent DRIVE");
    } else if (sscanf(line, "DRIVE %f %f %c", &a, &b, &extra) == 2) {
        if (!motion.drive(a, b, now)) Serial.println("ERR DRIVE expects armed state and two finite values in [-1,1]");
    } else if (sscanf(line, "PULSE %d %d %c", &channel, &width, &extra) == 2) {
        if (!hw::benchCalibration || motion.armed || faulted || !busReady || channel < 0 || channel > 15 ||
            width < hw::benchMinUs || width > hw::benchMaxUs) { Serial.println("ERR PULSE bench mode/range"); return; }
        if (!servos.off() || !servos.set(channel, width)) fault("PULSE I2C write failed");
        else { benchActive = true; lastBench = now; Serial.println("OK PULSE expires in 2 seconds"); }
    } else Serial.println("ERR command: STATUS ZERO ARM DRIVE turn side WALK STOP OFF PULSE channel us");
}

void setup() {
    Serial.begin(115200);
    Wire.begin(hw::sda, hw::scl, 400000); Wire.setTimeOut(20);
    busReady = servos.begin();
    imuReady = imu.beginI2C(hw::imuAddress, Wire) == BMI2_OK;
    if (imuReady) imuReady = imu.setAccelODR(BMI2_ACC_ODR_200HZ) == BMI2_OK &&
                             imu.setGyroODR(BMI2_GYR_ODR_200HZ) == BMI2_OK;
    lastImu = lastControl = millis();
    Serial.println("B9-1 manual-steering firmware; outputs disarmed; STATUS for readiness");
}

void loop() {
    static char line[96]; static size_t used = 0; static bool overflow = false;
    // Bounded work per loop: serial flooding must not starve control/timeout checks.
    for (int count = 0; count < 96 && Serial.available(); ++count) {
        const char c = char(Serial.read());
        if (c == '\n') {
            if (!overflow) { line[used] = 0; command(line); }
            else Serial.println("ERR line too long");
            used = 0; overflow = false;
        } else if (c != '\r' && !overflow) {
            if (used < sizeof(line)-1) line[used++] = c; else overflow = true;
        }
    }
    uint32_t now = millis();
    if (imuReady && !faulted) updateAttitude(now);
    if (motion.armed && now - lastImu > 100) fault("IMU data timeout");
    if (benchActive && now - lastBench > 2000) {
        benchActive = false;
        if (!servos.off()) fault("bench timeout I2C write failed");
    }
    if (now - lastControl >= 20) {
        const double dt = (now - lastControl) * .001;
        lastControl = now;
        const bool wasWalking = motion.walking;
        if (motion.tick(now, dt)) {
            // side is an OPERATOR request, not a measurement of lateral position.
            targets = b91::gait(motion.elapsed, b91::wrap(yaw - motion.headingTarget), -.05 * motion.side);
            if (!writeTargets()) fault("servo command I2C write failed");
        } else if (wasWalking) Serial.println("PAUSED command timeout/control overrun; send DRIVE then WALK to resume");
    }
    delay(1);
}
