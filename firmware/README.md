# PC-controlled walking

The PC calculates the gait. BMI270 data is displayed only. The controller ESP32 runs
`servo_camera/controller_esp32/controller_esp32.ino`, applies calibrated joint
commands and stops when the command stream is lost.

## Start

1. Upload `controller_esp32.ino` using Arduino IDE. Keep `measured_servos.h`,
   `servo_angles.h`, `motion_stream.h` and `wifi_config.h` in its sketch folder.
   Required libraries: Adafruit PWM Servo Driver and SparkFun BMI270.
2. Start the PC dashboard:

   ```bash
   .venv/bin/python -m pip install -r dashboard/requirements.txt
   .venv/bin/python dashboard/run.py
   ```

3. Open http://127.0.0.1:8765/, check the connection, then
   press Stop to release manual controls before selecting **Walk forward (PC)**.
   Support the robot for its first test. Stop disables PWM.

Walking uses the simulation's `flat` profile with zero heading/lateral correction.
Walking does not use IMU or camera input and has no IMU readiness, freshness
or tilt checks. It stops on browser heartbeat loss or when the ESP32 receives
no target frame for 300 ms. Restart
requires an explicit button press. Physical walking has not been validated.

Pitch uses the raised calibration frame: -90° to +45°, with +45° meaning foot
down. Yaw remains -45° to +45°. After changing measurements, run
`python3 servo_calibration/export_controller.py` and upload the sketch again.
The calibration GUI's export button writes the same header.

The camera ESP32 needs no changes. The separate USB walking firmware is removed.
Stop does not remove servo power; use the power switch if an I2C fault prevents
PWM from being disabled.
