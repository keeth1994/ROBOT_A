"""Fit a visible ramp side in camera rays after removing camera tilt and body tilt."""
import numpy as np


def fit_edge(rays):
    """Fit Y/X = slope * Z/X + intercept, ignoring gaps and small attachments."""
    if len(rays) < 8:
        return None
    rays = np.asarray(rays)
    fit = fit_section(rays)
    if fit is not None:
        return fit
    # A crest changes the projected side's direction. Fit its straight sections
    # separately rather than forcing one line through the whole outline.
    best, best_score = None, 0
    for start in range(0, len(rays) - 7, 16):
        section = rays[start:start + 32]
        fit = fit_section(section)
        if fit is None:
            continue
        slope, intercept, used = fit
        score = used.sum() * np.ptp(section[used, 2])
        if score > best_score:
            keep = np.zeros(len(rays), dtype=bool)
            keep[start:start + len(section)] = used
            best, best_score = (slope, intercept, keep), score
    return best


def fit_section(rays):
    if len(rays) < 8:
        return None
    z, y = rays[:, 2], rays[:, 1]
    if np.ptp(z) < .025:
        return None
    design = np.column_stack([z, np.ones(len(z))])
    keep = np.ones(len(z), dtype=bool)
    for _ in range(4):
        slope, intercept = np.linalg.lstsq(design[keep], y[keep], rcond=None)[0]
        residual = np.abs(y - design @ [slope, intercept])
        candidate = residual < max(.004, 2.5 * np.median(residual))
        if candidate.sum() < 8 or np.ptp(z[candidate]) < .025:
            return None
        keep = candidate
    if np.median(residual[keep]) > .008:
        return None
    slope, intercept = np.linalg.lstsq(design[keep], y[keep], rcond=None)[0]
    return float(slope), float(intercept), keep
