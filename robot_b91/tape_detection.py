import numpy as np
from scipy import ndimage


def detect_blue_tape(rgb):
    h, w = rgb.shape[:2]
    red, green, blue = rgb.astype(float).transpose(2, 0, 1)
    mask = (blue > 50) & (blue > red * 1.4) & (blue > green * 1.25)
    labels, _ = ndimage.label(mask)
    best_mask = np.zeros((h, w), dtype=bool)
    best_rows = []
    for label, bounds in enumerate(ndimage.find_objects(labels), start=1):
        if bounds is None:
            continue
        ys, _ = bounds
        if ys.stop - ys.start < h * 0.25 or ys.stop < h * 0.65:
            continue
        component = labels == label
        rows = []
        for y in range(ys.start, ys.stop, 2):
            indices = np.flatnonzero(component[y])
            if len(indices) < 3:
                continue
            left, right = int(indices[0]), int(indices[-1])
            width = right - left + 1
            if (
                left < 3
                or right > w - 4
                or width > w * 0.3
                or len(indices) < width * 0.9
            ):
                continue
            rows.append((y, left, right))
        if len(rows) < max(8, h * 0.08):
            continue
        edges = np.asarray(rows)
        if (
            np.max(np.diff(edges[:, 0])) > 6
            or np.median(np.abs(np.diff(edges[:, 1:], n=2, axis=0))) > 2
        ):
            continue
        if len(rows) > len(best_rows):
            best_mask, best_rows = component, rows
    return best_mask, best_rows
