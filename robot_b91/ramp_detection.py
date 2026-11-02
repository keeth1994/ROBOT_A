"""Find connected gray ramp surfaces and coherent side edges across exposures."""
import numpy as np
from scipy import ndimage


def detect_ramp(rgb, dark_min=3, dark_max=115, max_chroma=.28):
    original_h, original_w = rgb.shape[:2]
    stride = max(1, int(np.ceil(max(original_w / 160, original_h / 120))))
    pixels = rgb[::stride, ::stride].astype(float)
    gray = pixels.mean(axis=2)
    h, w = gray.shape
    smooth = ndimage.median_filter(gray, size=3)
    chroma = ndimage.median_filter(
        np.ptp(pixels, axis=2) / np.maximum(gray, 1), size=3)
    thresholds = np.unique(np.r_[dark_max, np.arange(65, 196, 10)])
    best_score, best_mask, best_rows = -np.inf, np.zeros((h, w), dtype=bool), []
    for threshold in thresholds:
        candidate = ((smooth > max(dark_min, threshold - 65)) &
                     (smooth < threshold) & (chroma < max_chroma))
        # Remove thin connections to shelves/rails before filling screw holes.
        candidate = ndimage.binary_opening(candidate, iterations=max(1, round(5 / stride)))
        labels, _ = ndimage.label(candidate)
        sizes = np.bincount(labels.ravel())
        for label in np.flatnonzero(sizes[1:] > h * w * .04) + 1:
            mask = labels == label
            ys, xs = np.nonzero(mask)
            if (np.median(ys) < h * .38 or not .12 * w < np.median(xs) < .88 * w or
                    np.ptp(ys) < h * .12):
                continue
            mask = ndimage.binary_fill_holes(mask)
            if np.median(mask.sum(axis=1)[mask.any(axis=1)]) < w * .18:
                continue
            rows, contrasts = [], []
            margin = max(7, w * .03)
            for y in range(int(ys.min()) + 2, int(ys.max()) - 1):
                x = np.flatnonzero(mask[y])
                if not len(x):
                    continue
                left, right = int(x[0]), int(x[-1])
                if right - left < w * .18 or left <= margin or right >= w - margin:
                    continue
                left_contrast = abs(smooth[y, left - 2] - smooth[y, left + 2])
                right_contrast = abs(smooth[y, right + 2] - smooth[y, right - 2])
                # Reject brightness variation inside the mat masquerading as edges.
                if min(left_contrast, right_contrast) < 8:
                    continue
                rows.append((y, left, right))
                contrasts.extend((left_contrast, right_contrast))
            ring = ndimage.binary_dilation(mask, iterations=3) & ~mask
            contrast = float(np.median(gray[ring]) - np.median(gray[mask])) if ring.any() else 0
            side_contrast = float(np.median(contrasts)) if contrasts else 0
            if abs(contrast) < 8 and side_contrast < 8:
                continue
            coherent_rows = continuous_edges(rows, h)
            if not coherent_rows and not visible_crest(mask, smooth):
                continue
            roughness = (float(np.median(np.abs(np.diff(np.asarray(rows)[:, 1:], n=2, axis=0))))
                         if len(rows) > 2 else 10 if rows else 0)
            score = (2 * len(rows) - 10 * roughness + .4 * min(side_contrast, 50) +
                     30 * np.sqrt(mask.mean()) + .15 * np.clip(contrast, 0, 60) -
                     .4 * np.std(smooth[mask]))
            if score > best_score:
                best_score, best_mask, best_rows = score, mask, coherent_rows
    mask = np.repeat(np.repeat(best_mask, stride, axis=0), stride, axis=1)[:original_h, :original_w]
    rows = [(y * stride, left * stride, right * stride)
            for y, left, right in best_rows]
    return mask, rows


def visible_crest(mask, gray):
    """Surface-only close-up needs a real crest, not a threshold in a gray wall."""
    h, w = mask.shape
    if mask[-3:].mean() < .65:
        return False
    columns = np.arange(w // 5, 4 * w // 5)
    heights = mask[:, columns].argmax(axis=0)
    valid = mask[:, columns].any(axis=0) & (heights >= 3) & (heights < h - 3)
    columns, heights = columns[valid], heights[valid]
    contrasts = np.abs(gray[heights - 2, columns] - gray[heights + 2, columns])
    return (len(heights) >= w * .25 and np.std(heights) < h * .12 and
            np.median(contrasts) >= 12)


def continuous_edges(rows, height):
    """Keep a coherent pair, excluding shelf attachments and clipped borders."""
    runs = []
    for row in rows:
        if not runs:
            runs.append([row])
            continue
        y, left, right = row
        py, pl, pr = runs[-1][-1]
        if y - py > 8 or max(abs(left - pl), abs(right - pr)) > 3 * (y - py) + 3:
            runs.append([])
        runs[-1].append(row)
    runs = [run for run in runs if len(run) >= 6 and run[-1][0] - run[0][0] >= height * .06]
    return max(runs, key=len) if runs else []
