"""Read-only visualization of the actual navigation frame and decision."""
import math
import numpy as np
from PIL import Image, ImageDraw


def navigation_status(navigation, now):
    """Report readiness without calling ready(), which updates controller telemetry."""
    age = None if navigation.last_seen is None else max(0., now - navigation.last_seen)
    if navigation.last_imu is None or now - navigation.last_imu > .05:
        return "HOLD REQUIRED", (255, 100, 80), "No fresh IMU sample", age
    if age is None or age > navigation.blind_timeout:
        return "HOLD REQUIRED", (255, 100, 80), "No valid camera measurement within timeout", age
    debug = navigation.debug
    if debug and debug["accepted"] and now - debug["timestamp"] <= .25:
        return "VALID CAMERA", (80, 255, 120), debug["reason"], age
    reason = debug["reason"] if debug else "Waiting for camera frame"
    return "USING LAST MEASUREMENT", (255, 200, 70), reason, age


def navigation_debug_image(navigation, now):
    """Return a 640x318 RGB panel. Only accepted rows receive a green midpoint."""
    panel = Image.new("RGB", (640, 318), (15, 18, 23))
    draw = ImageDraw.Draw(panel)
    draw.text((6, 4), "RGB: red=left  blue=right  green=midpoint", fill="white")
    draw.text((326, 4), "Mask: white=dark surface  cyan=used rows", fill="white")
    debug = navigation.debug
    if debug and debug["mask"] is not None:
        rgb = Image.fromarray(debug["rgb"].astype(np.uint8)).convert("RGB")
        mask = Image.fromarray((debug["mask"] * 255).astype(np.uint8)).convert("RGB")
        rgb_draw, mask_draw = ImageDraw.Draw(rgb), ImageDraw.Draw(mask)
        previous = None
        for i, (y, left, right) in enumerate(debug["rows"]):
            used = debug["accepted"] and i < len(debug["inliers"]) and debug["inliers"][i]
            midpoint = (left + right) / 2
            if used:
                rgb_draw.line((left, y, right, y), fill=(0, 65, 70))
                mask_draw.line((left, y, right, y), fill=(0, 145, 155))
                if previous is not None and y - previous[0] <= 4:
                    py, pl, pr = previous
                    rgb_draw.line((pl, py, left, y), fill=(255, 70, 65), width=2)
                    rgb_draw.line((pr, py, right, y), fill=(60, 140, 255), width=2)
                    rgb_draw.line(((pl+pr)/2, py, midpoint, y), fill=(60, 255, 100), width=2)
                for x, color in [(left, (255, 70, 65)), (right, (60, 140, 255)),
                                 (midpoint, (60, 255, 100))]:
                    rgb_draw.ellipse((x-1, y-1, x+1, y+1), fill=color)
                previous = (y, left, right)
            else:
                # Candidates rejected by geometry/quality gates are not valid edges.
                previous = None
                for x in [left, right]:
                    rgb_draw.line((x-2, y, x+2, y), fill=(255, 180, 40))
                    mask_draw.line((x-2, y, x+2, y), fill=(255, 180, 40))
        panel.paste(rgb.resize((320, 240)), (0, 22))
        panel.paste(mask.resize((320, 240)), (320, 22))
    else:
        draw.text((6, 60), "Waiting for a valid-size RGB frame", fill="white")
    status, color, reason, age = navigation_status(navigation, now)
    age_text = "never" if age is None else f"{age:.2f}s"
    rows = len(debug["rows"]) if debug else 0
    used = sum(debug["inliers"]) if debug and debug["accepted"] else 0
    draw.text((6, 266), f"{status} | camera age: {age_text} | used rows: {used}/{rows}", fill=color)
    draw.text((6, 282), f"Heading: {math.degrees(navigation.heading):+.2f} deg | lateral: {navigation.lateral*1000:+.1f} mm", fill="white")
    draw.text((6, 298), reason, fill=color)
    return np.array(panel)
