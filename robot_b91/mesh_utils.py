"""CAD mesh export helpers for B9-1."""

import numpy as np


def fmt(x):
    if np.isscalar(x):
        return f"{x:.9g}"
    return " ".join(f"{v:.9g}" for v in x)


def visual_mesh(record, origin):
    """Cluster overly dense tessellation per CAD solid, retaining real vertices.

    0.25 mm grid. Contacts/inertia never use this simplified visual geometry.
    Every retained vertex is from CAD; collapsed/duplicate faces are discarded.
    """
    v = np.array(record["vertices"], dtype=np.float64) - origin
    f = np.array(record["indices"], dtype=np.int64).reshape(-1, 3)
    if len(v) > 1200:
        cells = np.floor(v / 0.00025).astype(np.int64)
        _, first, inverse = np.unique(
            cells, axis=0, return_index=True, return_inverse=True
        )
        v = v[first]
        f = inverse[f]
        f = f[(f[:, 0] != f[:, 1]) & (f[:, 0] != f[:, 2]) & (f[:, 1] != f[:, 2])]
        _, keep = np.unique(np.sort(f, axis=1), axis=0, return_index=True)
        f = f[keep]
        cross = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
        f = f[np.linalg.norm(cross, axis=1) > 1e-14]
        used = np.unique(f)
        mapping = np.full(len(v), -1, dtype=np.int64)
        mapping[used] = np.arange(len(used))
        v = v[used]
        f = mapping[f]
    return v, f
