"""Talking to the ESP32 over serial: scanning and calibration."""

import csv
import os
import sys
import time


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


def cmd_calibrate(args, cfg):
    ser = open_port(args.port)
    print("Kalibreerimine: sisesta tilt-nurk (nt 60). Suuna LiDAR seinale ja otsi nurk,")
    print("mille juures kaugus on VÄIKSEIM: siis vaatab LiDAR seina suhtes risti (loodis).")
    print("Tühi rida lõpetab.")
    while True:
        s = input("tilt> ").strip()
        if not s:
            break
        ser.write(f"TILT {int(s)}\n".encode())
        ser.readline()
        ser.write(b"READ\n")
        line = ser.readline().decode(errors="replace").strip()
        if line.startswith("R,"):
            _, d, a = line.split(",")
            print(f"  kaugus {d} cm, tugevus {a}")
    ser.write(b"HOME\n")
