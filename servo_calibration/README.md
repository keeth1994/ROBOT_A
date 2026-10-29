# Servokalibrering

Last opp `esp32_calibration/esp32_calibration.ino` til ESP32, og installer
Adafruit PWM Servo Driver Library i Arduino IDE. Lukk Serial Monitor og start:

```bash
python3 -m pip install -r servo_calibration/requirements.txt
python3 servo_calibration/gui.py
```

Støtt roboten før aktivering: valgt servo går direkte til 1500 µs. Velg ledd og
kanal, juster pulsen og mål leddvinkelen før du lagrer. Knappene gir steg på
±1, ±5, ±10 og ±50 µs. Pulsområdet er 500–2500 µs; dette er ikke mekaniske grenser.

Yaw bruker -45°, 0° og +45° relativt til CAD. Pitch bruker:

| Kalibreringsvinkel | Stilling |
|---|---|
| -90° | 45° over horisontalt |
| -45° | Horisontalt ut |
| 0° | Halvveis opp fra CAD |
| +45° | Fot ned, CAD-null og øvre vinkelgrense |

Målingene lagres automatisk i `measurements.json`. **Eksporter ESP32-tabell**
skriver `firmware/servo_camera/controller_esp32/measured_servos.h`. Det samme
kan gjøres med `python3 servo_calibration/export_controller.py`.
Last opp `controller_esp32.ino` igjen etter eksport for vanlig styring og gange.
Se `firmware/README.md` for PC-dashboardet.
