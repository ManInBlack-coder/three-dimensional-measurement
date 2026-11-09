"""Talking to the ESP32 over serial: scanning and calibration."""

import csv
import os
import sys
import time

import numpy as np
from scipy.optimize import least_squares


# ======================================================================
# 6. Hardware: scanning and calibration
# ======================================================================
def open_port(port):
    try:
        import serial
    except ImportError:
        sys.exit("Vaja on pyserialit: pip install pyserial")
    ser = serial.Serial(port, 115200, timeout=2)
    time.sleep(2.0)  # ESP32 resets when the port is opened
    ser.reset_input_buffer()
    ser.write(b"PING\n")
    t0 = time.time()
    while time.time() - t0 < 5:
        if ser.readline().decode(errors="replace").strip() == "PONG":
            return ser
    sys.exit("Skanner ei vasta. Kontrolli porti ja kas lidar_skanner.ino on üles laaditud.")


def do_half_scan(ser, cfg, path):
    cmd = (f"SCAN {cfg['pan_min']} {cfg['pan_max']} {cfg['pan_step']} "
           f"{cfg['tilt_min']} {cfg['tilt_max']} {cfg['tilt_step']}\n")
    ser.write(cmd.encode())
    total, rows, t0 = None, [], time.time()
    while True:
        line = ser.readline().decode(errors="replace").strip()
        if not line:
            continue
        if line.startswith("START"):
            total = int(line.split(",")[1])
        elif line.startswith("P,"):
            _, p, t, d, a = line.split(",")
            rows.append((int(p), int(t), int(d), int(a)))
            if total and len(rows) % 50 == 0:
                el = time.time() - t0
                left = el / len(rows) * (total - len(rows))
                print(f"\r  {len(rows)}/{total} punkti, jäänud ~{left / 60:.1f} min   ", end="")
        elif line in ("DONE", "ABORTED"):
            break
    print()
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["pan", "tilt", "dist_cm", "amp"])
        w.writerows(rows)
    print(f"  Salvestatud {path} ({len(rows)} punkti)")


def cmd_scan(args, cfg):
    os.makedirs(args.kaust, exist_ok=True)
    ser = open_port(args.port)
    print(f"Asukoht '{args.nimi}'. Hoia skanner loodis ja ära liigu kiire ees.")
    input("Vajuta Enter, et alustada poolt A...")
    do_half_scan(ser, cfg, os.path.join(args.kaust, f"{args.nimi}_a.csv"))
    input("Pööra skanneri alust 180° (samas kohas!) ja vajuta Enter, et alustada poolt B...")
    do_half_scan(ser, cfg, os.path.join(args.kaust, f"{args.nimi}_b.csv"))
    print("Valmis. Liiguta skanner järgmisse kohta või käivita: python ruumala.py arvuta",
          args.kaust)


def fit_wall_sweep(angles, ranges_m):
    """Sweep across a flat wall: fits r = d / cos(s·(angle − c)) and returns (c, s, d).
    c = commanded angle pointing straight at the wall, s = real degrees per commanded degree."""
    a, r = np.asarray(angles, float), np.asarray(ranges_m, float)

    def resid(p):
        return p[2] / np.cos(np.radians(p[1] * (a - p[0]))) - r

    i = np.argmin(r)
    c, s, d = least_squares(resid, [a[i], 1.0, r[i]], loss="soft_l1", f_scale=0.03).x
    return c, abs(s), d


def calibration(pan_angles, pan_ranges, tilt_angles, tilt_ranges, lidar_offset):
    """Wall sweeps -> tilt_horizon, pan_scale, tilt_scale. Ranges in m, as the LiDAR reports them."""
    _, pan_scale, _ = fit_wall_sweep(pan_angles, np.asarray(pan_ranges) + lidar_offset)
    horizon, tilt_scale, _ = fit_wall_sweep(tilt_angles, np.asarray(tilt_ranges) + lidar_offset)
    return dict(tilt_horizon=horizon, pan_scale=pan_scale, tilt_scale=tilt_scale)


def _sweep(ser, axis, angles):
    out = []
    for a in angles:
        ser.write(f"{axis} {a}\n".encode())
        ser.readline()  # OK comes after the servo has settled
        ser.write(b"READ\n")
        line = ser.readline().decode(errors="replace").strip()
        if line.startswith("R,") and int(line.split(",")[1]) > 20:  # 0 = no valid reading
            out.append((a, int(line.split(",")[1]) / 100))
    return np.array(out).T


def cmd_calibrate(args, cfg):
    ser = open_port(args.port)
    print("Kalibreerimine. Pane skanner loodis umbes 1 m kaugusele tasasest seinast (vähemalt 2,5 m lai,")
    print("ees ei tohi olla mööblit) nii, et LiDAR vaatab seina poole. Kestab umbes poole minuti.")
    input("Vajuta Enter...")
    h = round(cfg["tilt_horizon"])
    ser.write(f"TILT {h}\nPAN 45\n".encode())
    ser.readline(), ser.readline()
    pan = _sweep(ser, "PAN", range(45, 136))
    ser.write(b"PAN 90\n")
    ser.readline()
    tilt = _sweep(ser, "TILT", range(max(h - 45, 0), min(h + 45, 180) + 1))
    ser.write(b"HOME\n")
    if pan.size == 0 or tilt.size == 0 or min(pan.shape[1], tilt.shape[1]) < 40:
        sys.exit("Liiga vähe mõõtmisi: kas LiDAR vaatab seina poole ja sein on lähemal kui 8 m?")
    cal = calibration(*pan, *tilt, cfg["lidar_offset"])
    print(f"Loodinurk {cal['tilt_horizon']:.1f}°, pan-skaala {cal['pan_scale']:.3f}, "
          f"tilt-skaala {cal['tilt_scale']:.3f}")
    if max(abs(cal["pan_scale"] - 1), abs(cal["tilt_scale"] - 1)) > 0.15:
        print("HOIATUS: skaala erineb 1-st üle 15 %. Kas sein on tasane ja vaba? Korda mõõtmist.")
    print("Kasuta neid arvutamisel, nt:\n  python ruumala.py "
          f"--tilt-horizon {cal['tilt_horizon']:.1f} --pan-skaala {cal['pan_scale']:.3f} "
          f"--tilt-skaala {cal['tilt_scale']:.3f} arvuta skaneeringud")
