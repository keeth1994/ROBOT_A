"""Find connected gray ramp surfaces and coherent side edges across exposures."""
import numpy as np
from PIL import Image
from scipy import ndimage


def detect_ramp(rgb, dark_min=3, dark_max=115, max_chroma=.28, include_single_edges=False,
                previous_mask=None):
    original_h, original_w = rgb.shape[:2]
    stride = max(1, int(np.ceil(max(original_w / 320, original_h / 240))))
    size = ((original_w + stride - 1) // stride, (original_h + stride - 1) // stride)
    pixels = np.asarray(Image.fromarray(rgb).resize(size, Image.Resampling.BOX)).astype(float)
    gray = pixels.mean(axis=2)
    h, w = gray.shape
    previous = (None if previous_mask is None else
                np.asarray(Image.fromarray(previous_mask).resize(size, Image.Resampling.NEAREST)))
    scale = max(w / 160, h / 120)
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
            if (not .12 * w < np.median(xs) < .88 * w or
                    np.ptp(ys) < h * .12):
                continue
            mask = ndimage.binary_fill_holes(mask)
            if np.median(mask.sum(axis=1)[mask.any(axis=1)]) < w * .18:
                continue
            rows, contrasts = [], []
            # Image borders are missing edges, never measured ramp boundaries.
            margin = 2
            for y in range(int(ys.min()) + 2, int(ys.max()) - 1):
                x = np.flatnonzero(mask[y])
                if not len(x):
                    continue
                left, right = int(x[0]), int(x[-1])
                if right - left < w * .18:
                    continue
                # Filling screw holes must not turn the floor between supports into mat.
                if candidate[y, left:right + 1].mean() < .8:
                    continue
                left_contrast = smooth[y, left - 2] - smooth[y, left + 2] if left > margin else 0
                right_contrast = smooth[y, right + 2] - smooth[y, right - 2] if right < w - margin else 0
                # Reject brightness variation inside the mat masquerading as edges.
                left = left if left_contrast >= 8 else None
                right = right if right_contrast >= 8 else None
                if left is None and right is None:
                    continue
                rows.append((y, left, right))
                contrasts.extend(c for c in (left_contrast, right_contrast) if c >= 8)
            ring = ndimage.binary_dilation(mask, iterations=3) & ~mask
            contrast = float(np.median(gray[ring]) - np.median(gray[mask])) if ring.any() else 0
            side_contrast = float(np.median(contrasts)) if contrasts else 0
            if abs(contrast) < 8 and side_contrast < 8:
                continue
            paired_rows = [row for row in rows if row[1] is not None and row[2] is not None]
            coherent_rows = continuous_edges(paired_rows, h, scale)
            independent_rows = independent_edges(rows, h, scale)
            clipped = contrast >= 8 and clipped_surface(mask)
            if not independent_rows and not visible_crest(mask, smooth) and not clipped:
                continue
            roughness = (float(np.median(np.abs(np.diff(np.asarray(coherent_rows)[:, 1:], n=2, axis=0)))) / scale
                         if len(coherent_rows) > 2 else 0)
            edge_count = sum((left is not None) + (right is not None) for _, left, right in independent_rows)
            score = (edge_count / scale - 10 * roughness + .4 * min(side_contrast, 50) +
                     30 * np.sqrt(mask.mean()) + .15 * np.clip(contrast, 0, 60) -
                     .4 * np.std(smooth[mask]))
            if include_single_edges:
                score += 30 * mask[3 * h // 4:].mean()
                if previous is not None:
                    score += 40 * (2 * (mask & previous).sum() / max(1, mask.sum() + previous.sum()))
            if score > best_score:
                best_score, best_mask = score, mask
                best_rows = independent_rows if include_single_edges else coherent_rows
    mask = np.asarray(Image.fromarray(best_mask).resize((original_w, original_h), Image.Resampling.NEAREST))
    rows = [(round((y + .5) * original_h / h - .5),
             None if left is None else round((left + .5) * original_w / w - .5),
             None if right is None else round((right + .5) * original_w / w - .5))
            for y, left, right in best_rows]
    return mask, rows


def detect_ramp_edges(rgb, dark_min=3, dark_max=115, max_chroma=.28, previous_mask=None):
    """Return visible sides independently; None marks a missing side."""
    return detect_ramp(rgb, dark_min, dark_max, max_chroma, include_single_edges=True,
                       previous_mask=previous_mask)


def independent_edges(rows, height, scale):
    """Keep continuous sides, including bends where the ramp changes slope."""
    kept = {}
    for side in (1, 2):
        runs = []
        for row in rows:
            y, x = row[0], row[side]
            if x is None:
                continue
            if not runs or y - runs[-1][-1][0] > 8 * scale:
                runs.append([])
            elif abs(x - runs[-1][-1][1]) > 3 * (y - runs[-1][-1][0]) + 3 * scale:
                runs.append([])
            runs[-1].append((y, x))
        for run in runs:
            if len(run) < 6 * scale or run[-1][0] - run[0][0] < height * .06:
                continue
            for y, x in run:
                kept.setdefault(y, [None, None])[side - 1] = x
    return [(y, *sides) for y, sides in sorted(kept.items())]


def clipped_surface(mask):
    """Keep surface visibility when a close-up fills both sides; do not invent edges."""
    h, w = mask.shape
    band = mask[h // 5:3 * h // 5]
    margin = max(1, w // 20)
    return (mask.mean() > .65 and band.mean() > .9 and
            band[:, :margin].mean() > .65 and band[:, -margin:].mean() > .65)


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


def continuous_edges(rows, height, scale=1):
    """Keep a coherent pair, excluding shelf attachments and clipped borders."""
    runs = []
    for row in rows:
        if not runs:
            runs.append([row])
            continue
        y, left, right = row
        py, pl, pr = runs[-1][-1]
        if y - py > 8 * scale or max(abs(left - pl), abs(right - pr)) > 3 * (y - py) + 3 * scale:
            runs.append([])
        runs[-1].append(row)
    runs = [run for run in runs if len(run) >= 6 * scale and run[-1][0] - run[0][0] >= height * .06]
    if not runs:
        return []
    rows = np.asarray(max(runs, key=len))
    design = np.column_stack([rows[:, 0], np.ones(len(rows))])
    keep = np.ones(len(rows), dtype=bool)
    for _ in range(4):
        fit = np.linalg.lstsq(design[keep], rows[keep, 1:], rcond=None)[0]
        residual = np.max(np.abs(rows[:, 1:] - design @ fit), axis=1)
        candidate = residual <= max(scale, 2.5 * np.median(residual))
        if candidate.sum() < 6 * scale:
            return []
        keep = candidate
    rows = rows[keep]
    if rows[-1, 0] - rows[0, 0] < height * .06:
        return []
    return [tuple(row) for row in rows.tolist()]
