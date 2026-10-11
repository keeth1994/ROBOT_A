"""Windows USB commissioning/operator console. Never arms or walks automatically."""
import argparse
import msvcrt
import time
import serial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", help="ESP32 serial port, e.g. COM3")
    args = ap.parse_args()
    print("z: stationary IMU zero | r: arm | w: walk/resume | space/s: hold | x: outputs off | q: quit")
    print("a/d: turn left/right | j/l: move left/right (repeat/hold key); release returns steering to zero")
    print("The operator supplies lateral steering. This is not autonomous ramp navigation.")
    with serial.Serial(args.port, 115200, timeout=0, write_timeout=.2) as port:
        armed = False
        turn = side = 0.
        turn_until = side_until = next_drive = next_status = 0.
        rx = bytearray()
        def send(text):
            port.write((text + "\n").encode("ascii"))
        try:
            while True:
                now = time.monotonic()
                if msvcrt.kbhit():
                    key = msvcrt.getwch().lower()
                    if key == "q": break
                    if key in (" ", "s"): send("STOP"); turn = side = 0
                    elif key == "x": send("OFF"); armed = False
                    elif key == "z": send("ZERO")
                    elif key == "r": send("ARM")
                    elif key == "w" and armed: send("DRIVE 0 0"); send("WALK")
                    elif key in ("a", "d"): turn = .5 if key == "a" else -.5; turn_until = now + .2
                    elif key in ("j", "l"): side = .5 if key == "j" else -.5; side_until = now + .2
                if now > turn_until: turn = 0
                if now > side_until: side = 0
                if armed and now >= next_drive:
                    send(f"DRIVE {turn} {side}"); next_drive = now + .1
                if now >= next_status: send("STATUS"); next_status = now + 1
                rx.extend(port.read(4096))
                while b"\n" in rx:
                    line, _, rest = rx.partition(b"\n"); rx = bytearray(rest)
                    text = line.decode("utf-8", errors="replace").strip()
                    if text == "OK ARM": armed = True
                    if text.startswith(("FAULT", "B9-1")) or text == "OK OFF": armed = False
                    print(text)
                if len(rx) > 8192: rx.clear()
                time.sleep(.01)
        finally:
            # Hold existing joint targets on disconnect; OFF is a separate deliberate command.
            send("STOP")


if __name__ == "__main__":
    main()
