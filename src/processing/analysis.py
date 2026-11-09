"""Floor, ceiling, floor plan, volumes and output files."""

import glob
import math
import os
import sys

import numpy as np
from scipy import ndimage

from .geometry import DepthView, find_floor_ceiling, load_scan_csv, level_scan, make_T, normals, rz, voxel_down
from .registration import register_all


# ======================================================================
# 4. Analysis: floor, ceiling, floor plan, volume, free air
# ======================================================================
def footprint(P, origins, ceil, res=0.025, band=(0.45, 0.08), close_m=0.25):
    xy = P[(P[:, 2] > ceil - band[0]) & (P[:, 2] < ceil - band[1]), :2]
    org = np.minimum(xy.min(axis=0), origins[:, :2].min(axis=0)) - 0.5
    top = np.maximum(xy.max(axis=0), origins[:, :2].max(axis=0)) + 0.5
    shape = tuple(np.ceil((top - org) / res).astype(int) + 1)
    walls = np.zeros(shape, bool)
    ij = np.floor((xy - org) / res).astype(int)
    walls[ij[:, 0], ij[:, 1]] = True
    seeds = np.floor((origins[:, :2] - org) / res).astype(int)
    near = int(0.5 / res)  # seed = biggest free area within 0.5 m: noisy ceiling hits above the scanner can block its own pixel
    st = ndimage.generate_binary_structure(2, 1)
    dist_to_wall = ndimage.distance_transform_edt(~walls)
    # Walls are "inflated" by r so that holes (windows, sparse areas) close up.
    # Then the interior is flood-filled starting from the scanner positions and
    # grown back by r up to the walls. If the interior "leaks" out, a larger r is tried.
    for cm in [close_m] + [c for c in (0.4, 0.6, 0.8, 1.0) if c > close_m]:
        r = cm / 2 / res
        lab, _ = ndimage.label(dist_to_wall > r, st)
        ids = set()
        for i, j in seeds:
            win = lab[max(i - near, 0):i + near + 1, max(j - near, 0):j + near + 1]
            if win.any():
                ids.add(np.bincount(win[win > 0]).argmax())
        core = np.isin(lab, list(ids))
        leak = core[0].any() or core[-1].any() or core[:, 0].any() or core[:, -1].any()
        if not leak:
            break
    inside = ndimage.distance_transform_edt(~core) <= r + 0.5
    area = inside.sum() * res * res
    fp = inside
    return dict(area=area, mask=fp, inside=inside, org=org, res=res, leak=leak, wall_xy=xy, close=cm)


def free_space(scans, fp, floor, ceil, vox=0.04, ang_step=2.0, tol=0.03):
    """Projective free-space computation: voxels behind which some scanner saw something."""
    res, org = fp["res"], fp["org"]
    ii, jj = np.nonzero(fp["mask"])
    x0, y0 = org + np.array([ii.min(), jj.min()]) * res
    x1, y1 = org + np.array([ii.max() + 1, jj.max() + 1]) * res
    xs = np.arange(x0 + vox / 2, x1, vox)
    ys = np.arange(y0 + vox / 2, y1, vox)
    zs = np.arange(floor + vox / 2, ceil, vox)
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    fi = np.clip(((X - org[0]) / res).astype(int), 0, fp["mask"].shape[0] - 1)
    fj = np.clip(((Y - org[1]) / res).astype(int), 0, fp["mask"].shape[1] - 1)
    in2d = fp["mask"][fi, fj]
    room = np.repeat(in2d[:, :, None], len(zs), axis=2)
    C = np.stack(np.meshgrid(xs, ys, zs, indexing="ij"), axis=-1)[room]

    free = np.zeros(len(C), bool)
    for s in scans:
        free |= DepthView(s, ang_step).seen_free(C, tol)

    # the boundary layer (walls, floor, ceiling) takes the value of its nearest interior neighbour
    grid_free = np.zeros(room.shape, bool)
    grid_free[room] = free
    margin = int(math.ceil(tol / vox)) + 1
    st = ndimage.generate_binary_structure(3, 1)
    interior = ndimage.binary_erosion(room, st, iterations=margin, border_value=0)
    _, ind = ndimage.distance_transform_edt(~interior, return_indices=True)
    nearest_free = grid_free[tuple(ind)]
    final_free = np.where(interior, grid_free, nearest_free) & room
    return dict(free_vol=final_free.sum() * vox ** 3, room_vox_vol=room.sum() * vox ** 3,
                free_grid=final_free, room=room, xs=xs, ys=ys, zs=zs)


# ======================================================================
# 5. Output: PLY, image, report
# ======================================================================
def write_ply(path, P, colors=None):
    with open(path, "w") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(P)}\nproperty float x\nproperty float y\nproperty float z\n")
        if colors is not None:
            f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write("end_header\n")
        for i, p in enumerate(P):
            line = f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}"
            if colors is not None:
                c = colors[i]
                line += f" {c[0]} {c[1]} {c[2]}"
            f.write(line + "\n")


def save_figure(path, scans, fp, fs, floor, ceil):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(13, 6))
    ax = fig.add_subplot(1, 2, 1)
    ext = [fp["org"][1], fp["org"][1] + fp["mask"].shape[1] * fp["res"],
           fp["org"][0], fp["org"][0] + fp["mask"].shape[0] * fp["res"]]
    ax.imshow(fp["mask"], origin="lower", extent=ext, cmap="Blues", alpha=0.35)
    # furniture/hidden in top view: how much of the height is not free
    occ = (fs["room"] & ~fs["free_grid"]).sum(axis=2).astype(float) * (fs["zs"][1] - fs["zs"][0])
    occ[occ < 0.08] = np.nan
    ax.imshow(occ, origin="lower", cmap="Oranges",
              extent=[fs["ys"][0], fs["ys"][-1], fs["xs"][0], fs["xs"][-1]], alpha=0.8)
    w = fp["wall_xy"]
    ax.scatter(w[:, 1], w[:, 0], s=0.3, c="k")
    for s in scans:
        if s.name.endswith("_a"):
            o = s.origin
            ax.plot(o[1], o[0], "r^", ms=10)
            ax.annotate(s.name[:-2], (o[1], o[0]), xytext=(5, 5), textcoords="offset points",
                        color="r")
    ax.set_title("Põhiplaan (sinine), mööbel/varjatud (oranž), skanner (▲)")
    ax.set_xlabel("y (m)")
    ax.set_ylabel("x (m)")
    ax.set_aspect("equal")

    ax3 = fig.add_subplot(1, 2, 2, projection="3d")
    P = voxel_down(np.vstack([s.world for s in scans]), 0.06)
    P = P[(P[:, 2] < ceil - 0.05)]
    ax3.scatter(P[:, 0], P[:, 1], P[:, 2], s=0.5, c=P[:, 2], cmap="viridis")
    ax3.set_title("Punktipilv (lagi peidetud)")
    ax3.set_box_aspect((np.ptp(P[:, 0]), np.ptp(P[:, 1]), np.ptp(P[:, 2])))
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def analyze(folder, cfg, out_dir=None, vox=0.04):
    out_dir = out_dir or folder
    log_lines = []

    def log(msg=""):
        print(msg)
        log_lines.append(msg)

    files = sorted(glob.glob(os.path.join(folder, "*_a.csv")) + glob.glob(os.path.join(folder, "*_b.csv")))
    if not files:
        sys.exit(f"Kaustas '{folder}' pole faile kujul <asukoht>_a.csv / <asukoht>_b.csv")

    log(f"Laen {len(files)} skaneeringut kaustast {folder}")
    scans = [load_scan_csv(f, cfg) for f in files]
    for s in scans:
        tilt = level_scan(s)
        msg = f"  {s.name}: {len(s.local)} punkti"
        msg += ", lagi ei olnud selgelt näha (loodimata)" if math.isnan(tilt) else f", kalle {tilt:.1f}°"
        log(msg)

    log("\nSkaneeringute kokkusobitamine:")
    scans = register_all(scans, log)
    P = np.vstack([s.world for s in scans])
    origins = np.array([s.origin for s in scans])

    # rotate so that the walls are parallel to the axes in the figure
    D = voxel_down(P, 0.05)
    n = normals(D)
    wn = n[np.abs(n[:, 2]) < 0.3]
    ang = np.degrees(np.arctan2(wn[:, 1], wn[:, 0])) % 90
    h, e = np.histogram(ang, bins=90, range=(0, 90))
    rot = make_T(rz(-math.radians(e[np.argmax(ndimage.uniform_filter1d(h, 3, mode="wrap"))] + 0.5)))
    for s in scans:
        s.T = rot @ s.T
    P = np.vstack([s.world for s in scans])

    floor, ceil = find_floor_ceiling(P)
    P[:, 2] -= floor
    for s in scans:
        s.T = make_T(t=np.array([0, 0, -floor])) @ s.T
    ceil -= floor
    floor = 0.0
    origins = np.array([s.origin for s in scans])

    fp = footprint(P, origins, ceil)
    height = ceil - floor
    room_vol = fp["area"] * height
    fs = free_space(scans, fp, floor, ceil, vox=vox)
    obj_vol = max(0.0, room_vol - fs["free_vol"] * room_vol / max(fs["room_vox_vol"], 1e-9))
    free_vol = room_vol - obj_vol

    log("\n================ TULEMUS ================")
    log(f"Lae kõrgus:            {height:6.2f} m")
    log(f"Põranda pindala:       {fp['area']:6.2f} m²")
    log(f"TOA RUUMALA:           {room_vol:6.2f} m³")
    log(f"Vaba õhk:              {free_vol:6.2f} m³")
    log(f"Mööbel + varjatud ala: {obj_vol:6.2f} m³ ({obj_vol / room_vol * 100:.1f}% toast)")
    log("=========================================")
    if fp["leak"]:
        log("HOIATUS: põhiplaan 'lekib' välja: seintes lae lähedal on suur auk")
        log("         (avatud uks kuni laeni, suur aken?). Tulemus ei ole usaldusväärne.")
    elif fp["close"] > 0.3:
        log(f"Märkus: seintes olid augud, need suleti kuni {fp['close']*100:.0f} cm ulatuses.")
    log("Märkus: 'mööbel + varjatud' sisaldab ka alasid, mida ükski skanner ei näinud.")
    log("        Rohkem asukohti = väiksem varjatud ala = täpsem mööbli ruumala.")

    os.makedirs(out_dir, exist_ok=True)
    write_ply(os.path.join(out_dir, "tuba.ply"), voxel_down(P, 0.02))
    save_figure(os.path.join(out_dir, "tuba.png"), scans, fp, fs, floor, ceil)
    with open(os.path.join(out_dir, "tulemus.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(log_lines) + "\n")
    log(f"\nSalvestatud: {out_dir}/tuba.png, tuba.ply, tulemus.txt")
    return dict(height=height, area=fp["area"], room=room_vol, free=free_vol, objects=obj_vol)
