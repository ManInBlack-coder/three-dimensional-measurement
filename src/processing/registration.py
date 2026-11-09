"""Aligning the scans from different positions into one coordinate system."""

import math

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree

from .geometry import DepthView, apply, find_floor_ceiling, make_T, normals, rz, voxel_down


# ======================================================================
# 3. Registration (aligning the scans)
# ======================================================================
def icp_4dof(src, tgt, tgt_n, T0, iters=40, max_d=(0.4, 0.2, 0.1, 0.06), dof=(0, 1, 2, 3)):
    """Point-to-plane ICP: rotation about the z axis + translation (scanner is level).
    dof selects the degrees of freedom: 0 = rotation, 1-3 = translation x, y, z."""
    tree = cKDTree(tgt)
    dof = list(dof)
    T = T0.copy()
    stages = np.repeat(max_d, max(1, iters // len(max_d)))
    for md in stages:
        P = apply(T, src)
        d, idx = tree.query(P, distance_upper_bound=md)
        ok = np.isfinite(d)
        if ok.sum() < 30:
            break
        p, q, n = P[ok], tgt[idx[ok]], tgt_n[idx[ok]]
        r = np.sum(n * (p - q), axis=1)
        J = np.stack([n[:, 0] * -p[:, 1] + n[:, 1] * p[:, 0], n[:, 0], n[:, 1], n[:, 2]], axis=1)
        J = J[:, dof]
        w = 1.0 / np.maximum(1.0, np.abs(r) / 0.02)  # Huber weight
        A = J.T @ (J * w[:, None]) + 1e-6 * np.eye(len(dof))
        x = np.zeros(4)
        x[dof] = np.linalg.solve(A, -J.T @ (r * w))
        T = make_T(rz(x[0]), x[1:]) @ T
        if np.abs(x).max() < 1e-5:
            break
    P = apply(T, src)
    d, _ = tree.query(P)
    inlier = float(np.mean(d < 0.05))
    rmse = float(np.sqrt(np.mean(np.minimum(d, 0.05) ** 2)))
    return T, inlier, rmse


def band_points(P, band=(0.45, 0.08)):
    """Points near the ceiling (above the furniture), in top view."""
    z = P[:, 2]
    ceil = find_floor_ceiling(P)[1]
    sel = (z > ceil - band[0]) & (z < ceil - band[1])
    return P[sel, :2]


def occupancy(xy, res, origin, shape):
    g = np.zeros(shape)
    ij = np.floor((xy - origin) / res).astype(int)
    ok = (ij[:, 0] >= 0) & (ij[:, 1] >= 0) & (ij[:, 0] < shape[0]) & (ij[:, 1] < shape[1])
    g[ij[ok, 0], ij[ok, 1]] = 1
    return ndimage.gaussian_filter(g, 1.2)


def global_align(src_pts, tgt_pts, res=0.05, yaw_step=2.0, top=4):
    """Finds the best rotation and shift by overlapping the walls in top view."""
    s_xy, t_xy = band_points(src_pts), band_points(tgt_pts)
    if len(s_xy) < 50 or len(t_xy) < 50:
        return []
    ext = max(np.ptp(s_xy, axis=0).max(), np.ptp(t_xy, axis=0).max())
    N = int(2 ** math.ceil(math.log2(2 * ext / res + 8)))
    t_org = t_xy.min(axis=0) - 2 * res
    T_img = occupancy(t_xy, res, t_org, (N, N))
    FT = np.fft.rfft2(T_img)
    cands = []
    for yaw_deg in np.arange(0, 360, yaw_step):
        R2 = rz(math.radians(yaw_deg))[:2, :2]
        rs = s_xy @ R2.T
        s_org = rs.min(axis=0) - 2 * res
        S_img = occupancy(rs, res, s_org, (N, N))
        corr = np.fft.irfft2(FT * np.conj(np.fft.rfft2(S_img)), s=(N, N))
        k = np.unravel_index(np.argmax(corr), corr.shape)
        shift = np.array([k[0] if k[0] < N // 2 else k[0] - N,
                          k[1] if k[1] < N // 2 else k[1] - N]) * res
        t2 = t_org + shift - s_org
        cands.append((corr[k] / (S_img.sum() + 1e-9), yaw_deg, t2))
    cands.sort(key=lambda c: -c[0])
    out, used = [], []
    for score, yaw_deg, t2 in cands:
        if any(abs(((yaw_deg - u + 180) % 360) - 180) < 10 for u in used):
            continue
        used.append(yaw_deg)
        out.append(make_T(rz(math.radians(yaw_deg)), np.array([t2[0], t2[1], 0.0])))
        if len(out) >= top:
            break
    return out


def _yaw(T):
    return math.degrees(math.atan2(T[1, 0], T[0, 0]))


def _set_z(T0, src, ceil_t):
    T0[2, 3] = ceil_t - np.percentile(apply(T0, src)[:, 2], 99.5)
    return T0


def _walls(P):
    n = normals(P)
    return P[np.abs(n[:, 2]) < 0.3]


def pose_score(src_w, tgt_w_tree, T, views=()):
    """How well the walls overlap where the clouds meet, minus contradictions:
    wall points that would land in an area some scanner saw as empty.
    Floor and ceiling are left out, because they match at any rotation."""
    P = apply(T, src_w)
    d, _ = tgt_w_tree.query(P, distance_upper_bound=0.3)
    near = np.isfinite(d)
    n30, n5 = near.sum(), (d < 0.05).sum()
    if n30 < 30:
        return 0.0
    score = (n5 / n30) * min(1.0, n30 / (0.25 * len(src_w)))
    if views:
        bad = np.zeros(len(P), bool)
        for v in views:
            bad |= v.seen_free(P, tol=0.15)
        score -= 2.0 * bad.mean()
    return score


def _half_rel(delta_deg):
    return make_T(rz(math.radians(180 + delta_deg)))


def _fit_b(pos, tgt_world=None):
    """Refines the rotation of half b about the shared axis (shift is 0, since the base turned in place).
    Scores wall overlap and contradictions: points of b must not lie in an area
    that half a saw as empty (and vice versa)."""
    a, b = pos["a"], pos["b"]
    parts = [a.local]
    if tgt_world is not None and len(tgt_world):
        parts.append(apply(np.linalg.inv(a.T), tgt_world))
    tgt = voxel_down(np.vstack(parts), 0.03)
    tgt_n = normals(tgt)
    tree_w = cKDTree(tgt[np.abs(tgt_n[:, 2]) < 0.3])
    src = voxel_down(b.local, 0.04)
    src_w = _walls(src)
    views = [DepthView(points=a.local, origin=a.origin_local)]
    b_view_pts = b.local
    a_w = _walls(voxel_down(a.local, 0.04))

    def score(T):
        s = pose_score(src_w, tree_w, T, views)
        # reverse check: walls of a inside the area b saw as empty
        vb = DepthView(points=apply(T, b_view_pts), origin=apply(T, b.origin_local[None])[0])
        return s - 2.0 * vb.seen_free(a_w, tol=0.15).mean()

    cands = sorted(((score(_half_rel(d)), d) for d in np.arange(-15, 15.5, 1.0)), reverse=True)
    best = None
    for _, d in cands[:3]:
        T, _, _ = icp_4dof(src, tgt, tgt_n, _half_rel(d), dof=(0,), max_d=(0.15, 0.08, 0.05))
        sc = score(T)
        if best is None or sc > best[1]:
            best = (T, sc)
    pos["rel"] = best[0]
    b.T = a.T @ best[0]
    return best[1]


def _pos_cloud(pos):
    pts = [voxel_down(pos["a"].local, 0.04)]
    if "b" in pos:
        pts.append(apply(pos["rel"], voxel_down(pos["b"].local, 0.04)))
    return np.vstack(pts)


def _place(pos):
    pos["a"].T = pos["T"]
    if "b" in pos:
        pos["b"].T = pos["T"] @ pos["rel"]


def register_all(scans, log, rounds=2):
    """Places all scans into a common coordinate system.

    One position = halves a and b, which share the same rotation axis and differ
    by a ~180° rotation (the exact rotation is computed). Positions are added
    greedily: at each step, the one that best overlaps the already placed cloud.
    Initial guesses come from FFT correlation of top-view walls, refined with ICP.
    """
    positions = {}
    for s in scans:
        base, half = s.name.rsplit("_", 1)
        positions.setdefault(base, {})[half if half in ("a", "b") else "a"] = s
    for name, pos in positions.items():
        if "a" not in pos:            # only b present: use it as a
            pos["a"] = pos.pop("b")
        pos["rel"] = _half_rel(0)
        pos["name"] = name

    order = sorted(positions)
    ref = positions[order[0]]
    ref["T"] = np.eye(4)
    _place(ref)
    if "b" in ref:
        _fit_b(ref)
        ref["b_done"] = True
    placed, remaining = [ref], [positions[n] for n in order[1:]]
    log(f"  {ref['name']}: tugipunkt")

    def map_cloud(exclude=None, ref_only_a=True):
        parts = []
        for p in placed:
            if p is exclude:
                continue
            if p is ref and ref_only_a and not ref.get("b_done"):
                parts.append(apply(p["T"], voxel_down(p["a"].local, 0.04)))
            else:
                parts.append(apply(p["T"], _pos_cloud(p)))
        return voxel_down(np.vstack(parts), 0.04) if parts else np.zeros((0, 3))

    while remaining:
        tgt = map_cloud()
        tgt_n = normals(tgt)
        ceil_t = np.percentile(tgt[:, 2], 99.5)
        views = [DepthView(p[h]) for p in placed for h in ("a", "b")
                 if h in p and not (p is ref and h == "b" and not ref.get("b_done"))]
        tgt_w = tgt[np.abs(tgt_n[:, 2]) < 0.3]
        tree_w = cKDTree(tgt_w)
        best = None
        for pos in remaining:
            pos["rel"] = _half_rel(0)
            src = _pos_cloud(pos)
            for T0 in global_align(src, tgt, top=6):
                _set_z(T0, src, ceil_t)
                T, _, _ = icp_4dof(src, tgt, tgt_n, T0)
                if "b" in pos:
                    # refine the rotation between halves for this guess, then ICP again
                    pos["T"] = T
                    _place(pos)
                    _fit_b(pos, tgt)
                    src2 = _pos_cloud(pos)
                    T, _, _ = icp_4dof(src2, tgt, tgt_n, T)
                else:
                    src2 = src
                sc = pose_score(_walls(src2), tree_w, T, views)
                # reverse check: existing walls inside the area the new position saw as empty
                own = [DepthView(points=apply(T @ (np.eye(4) if h == "a" else pos["rel"]), pos[h].local),
                                 origin=apply(T @ (np.eye(4) if h == "a" else pos["rel"]),
                                              pos[h].origin_local[None])[0])
                       for h in ("a", "b") if h in pos]
                bad = np.zeros(len(tgt_w), bool)
                for v in own:
                    bad |= v.seen_free(tgt_w, tol=0.15)
                sc -= 2.0 * bad.mean()
                if best is None or sc > best[2]:
                    best = (pos, pos["rel"].copy(), sc, T)
            pos["rel"] = _half_rel(0)
        pos, rel, inl, T = best
        pos["rel"], pos["T"] = rel, T
        _place(pos)
        warn = "  <- KONTROLLI!" if inl < 0.2 else ""
        log(f"  {pos['name']}: pööre {_yaw(T):6.1f}°, asukoht ({T[0,3]:.2f}, {T[1,3]:.2f}) m, "
            f"seinte kattuvus {inl*100:4.1f}%{warn}")
        placed.append(pos)
        remaining.remove(pos)

    # refinement: each position against all others, then each b half
    for _ in range(rounds):
        for pos in placed[1:]:
            others = map_cloud(exclude=pos)
            pos["T"], _, _ = icp_4dof(_pos_cloud(pos), others, normals(others), pos["T"],
                                      max_d=(0.1, 0.06, 0.04))
            _place(pos)
        for pos in placed:
            if "b" in pos:
                _fit_b(pos, map_cloud(exclude=pos))
    for pos in placed:
        if "b" in pos:
            d = _yaw(pos["rel"]) % 360 - 180
            log(f"  {pos['name']}: poolte vahe 180° {d:+.1f}°")
    return [p[h] for p in placed for h in ("a", "b") if h in p]
