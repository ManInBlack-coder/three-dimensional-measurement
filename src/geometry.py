"""Raw measurements -> 3D points, plus geometry helpers shared by registration and analysis."""

import math
import os

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree


def rz(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def make_T(R=np.eye(3), t=np.zeros(3)):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def apply(T, P):
    return P @ T[:3, :3].T + T[:3, 3]


# ======================================================================
# 1. Raw data -> 3D points
# ======================================================================
class Scan:
    """One half-scan: points in scanner coordinates + pose in the world."""

    def __init__(self, name, points, origin_local):
        self.name = name
        self.local = points            # (N,3) in the scanner's own coordinates
        self.origin_local = origin_local
        self.T = np.eye(4)             # scanner -> world

    @property
    def world(self):
        return apply(self.T, self.local)

    @property
    def origin(self):
        return apply(self.T, self.origin_local[None])[0]


def raw_to_points(pan, tilt, dist_cm, amp, cfg):
    r = dist_cm / 100.0
    ok = (r >= cfg["min_range"]) & (r <= cfg["max_range"]) & (amp >= cfg["min_amp"])
    th = np.radians(pan[ok])
    ph = np.radians((tilt[ok] - cfg["tilt_horizon"]) * cfg["tilt_dir"])
    rr = r[ok] + cfg["lidar_offset"]
    d = np.stack([np.cos(ph) * np.cos(th), np.cos(ph) * np.sin(th), np.sin(ph)], axis=1)
    axis = np.array([0, 0, cfg["axis_height"]])
    return axis + rr[:, None] * d, axis


def load_scan_csv(path, cfg):
    data = np.genfromtxt(path, delimiter=",", names=True)
    pts, origin = raw_to_points(data["pan"], data["tilt"], data["dist_cm"], data["amp"], cfg)
    name = os.path.splitext(os.path.basename(path))[0]
    return Scan(name, pts, origin)


# ======================================================================
# 2. Helpers: normals, downsampling, levelling
# ======================================================================
def voxel_down(P, size):
    keys = np.floor(P / size).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return P[np.sort(idx)]


def normals(P, k=12):
    tree = cKDTree(P)
    _, idx = tree.query(P, k=min(k, len(P)))
    nb = P[idx]
    nb = nb - nb.mean(axis=1, keepdims=True)
    cov = np.einsum("nki,nkj->nij", nb, nb)
    _, vec = np.linalg.eigh(cov)
    n = vec[:, :, 0]
    return n / np.linalg.norm(n, axis=1, keepdims=True)


def ransac_plane(P, thresh=0.02, iters=300, rng=None):
    rng = rng or np.random.default_rng(0)
    best_n, best_cnt = None, -1
    for _ in range(iters):
        a, b, c = P[rng.choice(len(P), 3, replace=False)]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        cnt = np.sum(np.abs((P - a) @ n) < thresh)
        if cnt > best_cnt:
            best_cnt, best_n, best_p = cnt, n, a
    inl = np.abs((P - best_p) @ best_n) < thresh
    # least-squares refinement
    Q = P[inl]
    c = Q.mean(axis=0)
    _, _, vt = np.linalg.svd(Q - c)
    n = vt[2]
    return n, c, inl


def level_scan(scan):
    """Levels the scan using the ceiling (points high above the scanner)."""
    P = scan.local
    rel = P - scan.origin_local
    elev = np.degrees(np.arctan2(rel[:, 2], np.hypot(rel[:, 0], rel[:, 1])))
    up = P[elev > 55]
    if len(up) < 50:
        return 0.0
    n, _, inl = ransac_plane(up, thresh=0.03)
    if n[2] < 0:
        n = -n
    tilt = math.degrees(math.acos(min(1.0, n[2])))
    if tilt > 10 or inl.mean() < 0.5:
        return float("nan")  # ceiling not clear, leave unlevelled
    axis = np.cross(n, [0, 0, 1.0])
    s = np.linalg.norm(axis)
    if s < 1e-9:
        return 0.0
    axis /= s
    ang = math.asin(min(1.0, s))
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    R = np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K
    scan.local = (scan.local - scan.origin_local) @ R.T + scan.origin_local
    return tilt


class DepthView:
    """A scan's "depth image": the distance measured in each direction.
    Used to tell whether a point lies in an area the scanner saw as empty."""

    def __init__(self, scan=None, step=2.0, points=None, origin=None):
        self.o = scan.origin if scan is not None else origin
        pts = scan.world if scan is not None else points
        self.step = step
        self.ab, self.eb = int(round(360 / step)), int(round(180 / step))
        rel = pts - self.o
        az, el, rng = self._dir(rel)
        depth = np.full((self.ab, self.eb), np.inf)
        np.minimum.at(depth, (az, el), rng)
        filled = ndimage.minimum_filter(np.where(np.isinf(depth), 1e9, depth), size=3, mode="wrap")
        depth = np.where(np.isinf(depth), filled, depth)
        depth[depth >= 1e9] = -1          # direction with no measurement
        self.depth = depth

    def _dir(self, rel):
        rng = np.linalg.norm(rel, axis=1)
        az = ((np.degrees(np.arctan2(rel[:, 1], rel[:, 0])) % 360) // self.step).astype(int) % self.ab
        el = np.clip((np.degrees(np.arcsin(rel[:, 2] / np.maximum(rng, 1e-9))) + 90) // self.step,
                     0, self.eb - 1).astype(int)
        return az, el, rng

    def seen_free(self, P, tol):
        az, el, rng = self._dir(P - self.o)
        d = self.depth[az, el]
        return (d > 0) & (rng < d - tol)
