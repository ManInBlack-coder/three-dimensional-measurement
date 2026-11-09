"""Simulated room, for testing without hardware."""

import csv
import math
import os

import numpy as np

from geometry import rz


# ======================================================================
# 7. Simulator (for testing without hardware)
# ======================================================================
SIM_ROOM = dict(
    size=(4.6, 3.7), height=2.55,
    column=((4.0, 0.0, 4.6, 0.7),),   # column in the corner (part of the building, not furniture)
    furniture=[                        # (x0, y0, z0, x1, y1, z1)
        (0.0, 0.0, 0.0, 1.2, 0.6, 2.05),   # wardrobe
        (1.6, 2.75, 0.0, 3.6, 3.7, 0.85),  # sofa
        (0.3, 1.5, 0.0, 1.1, 2.6, 0.75),   # chest of drawers/table
        (3.3, 1.2, 0.0, 3.8, 1.7, 1.1),    # shelf
    ],
)


def sim_raycast(O, D, room):
    W, L = room["size"]
    H = room["height"]
    lo, hi = np.array([0, 0, 0.0]), np.array([W, L, H])
    with np.errstate(divide="ignore", invalid="ignore"):
        inv = 1.0 / D
    t_exit = np.min(np.maximum((lo - O) * inv, (hi - O) * inv), axis=1)
    boxes = list(room["furniture"]) + [(c[0], c[1], 0.0, c[2], c[3], H) for c in room["column"]]
    t = t_exit.copy()
    for b in boxes:
        blo, bhi = np.array(b[:3]), np.array(b[3:])
        t1, t2 = (blo - O) * inv, (bhi - O) * inv
        tn = np.max(np.minimum(t1, t2), axis=1)
        tf = np.min(np.maximum(t1, t2), axis=1)
        hit = (tf >= tn) & (tn > 0)
        t = np.where(hit & (tn < t), tn, t)
    return t


def sim_truth(room):
    W, L = room["size"]
    H = room["height"]
    col = sum((c[2] - c[0]) * (c[3] - c[1]) for c in room["column"])
    area = W * L - col
    furn = sum((b[3] - b[0]) * (b[4] - b[1]) * (b[5] - b[2]) for b in room["furniture"])
    return dict(area=area, room=area * H, objects=furn, free=area * H - furn)


def cmd_simulate(args, cfg):
    rng = np.random.default_rng(args.seed)
    room = SIM_ROOM
    positions = [(2.3, 1.6), (3.6, 2.3), (1.9, 0.9), (0.8, 3.1)][: args.asukohti]
    os.makedirs(args.kaust, exist_ok=True)
    pans = np.arange(cfg["pan_min"], cfg["pan_max"] + 1, cfg["pan_step"])
    tilts = np.arange(cfg["tilt_min"], cfg["tilt_max"] + 1, cfg["tilt_step"])
    PP, TT = np.meshgrid(pans, tilts, indexing="ij")
    PP, TT = PP.ravel().astype(float), TT.ravel().astype(float)
    for k, (x, y) in enumerate(positions, 1):
        yaw = rng.uniform(0, 360)
        tilt_err = rng.normal(0, 0.8, 2)  # scanner is not perfectly level
        delta = rng.normal(0, 5)
        for half, extra in (("a", 0.0), ("b", 180 + delta)):
            shift = rng.normal(0, 0.01, 2) if half == "b" else np.zeros(2)
            R = (rz(math.radians(yaw + extra))
                 @ np.array(_rot_xy(math.radians(tilt_err[0]), math.radians(tilt_err[1]))))
            base = np.array([x + shift[0], y + shift[1], 1.20])
            th = np.radians(PP)
            ph = np.radians((TT - cfg["tilt_horizon"]) * cfg["tilt_dir"])
            d = np.stack([np.cos(ph) * np.cos(th), np.cos(ph) * np.sin(th), np.sin(ph)], 1)
            axis_w = base + R @ np.array([0, 0, cfg["axis_height"]])
            dw = d @ R.T
            t = sim_raycast(axis_w, dw, room) - cfg["lidar_offset"]
            dist = (t + rng.normal(0, 0.015, len(t))) * 100
            amp = np.where(rng.random(len(t)) < 0.02, 50, 800)  # 2% weak readings
            path = os.path.join(args.kaust, f"asukoht{k}_{half}.csv")
            with open(path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["pan", "tilt", "dist_cm", "amp"])
                for row in zip(PP.astype(int), TT.astype(int), np.round(dist).astype(int), amp):
                    w.writerow(row)
        print(f"  asukoht{k}: ({x}, {y}), pööre {yaw:.0f}°, poolte vahe 180° {delta:+.1f}°")
    tr = sim_truth(room)
    print(f"Simuleeritud tuba: pindala {tr['area']:.2f} m², ruumala {tr['room']:.2f} m³, "
          f"mööbel {tr['objects']:.2f} m³, vaba õhk {tr['free']:.2f} m³")
    print(f"Andmed kaustas {args.kaust}. Analüüs: python ruumala.py arvuta {args.kaust}")
    return tr


def _rot_xy(ax, ay):
    cx, sx, cy, sy = math.cos(ax), math.sin(ax), math.cos(ay), math.sin(ay)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    return Ry @ Rx
