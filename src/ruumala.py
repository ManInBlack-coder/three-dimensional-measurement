#!/usr/bin/env python3
"""
Room volume measurement with a LiDAR scanner (even with furniture in the room)
==============================================================================

How it works
------------
1. The scanner (ESP32 + TF-Luna + 2 servos) measures two halves at each position:
   half "a" and half "b"; in between you rotate the scanner base 180° by hand.
2. Each measurement (pan, tilt, distance) is converted into a 3D point.
3. Each scan is levelled using the ceiling (a small tilt doesn't spoil the result).
4. Positions are found automatically, scanner positions don't need to be measured:
   - in top view, search for the rotation and shift at which the walls overlap
     (FFT correlation), then refine with ICP (rotation about the vertical axis
     + shift in x, y, z, since the scanner stands level);
   - a wrong placement is detected by contradictions: a wall must not be in a
     place that another scanner saw as empty;
   - the exact angle between halves A and B (about 180°) is computed.
5. Analysis:
   - floor and ceiling = the lowest and highest horizontal surface
   - floor plan = wall points near the ceiling, above the furniture; this way the
     walls automatically "extend" behind cabinets and sofas too
   - ROOM VOLUME = floor plan area × height
   - FREE AIR: the room is divided into cubes (voxels). A cube is free if some
     scan sees past it. The rest is furniture or hidden area.
     The more positions you scan from, the less hidden area remains.

Commands
--------
  python ruumala.py simuleeri                 # test data without hardware
  python ruumala.py arvuta skaneeringud       # analyse a folder
  python ruumala.py kalibreeri --port COM3    # find the tilt servo level angle
  python ruumala.py skanni --port COM3 --nimi asukoht1   # scan one position

Dependencies: pip install numpy scipy matplotlib pyserial
The resulting 3D file (tuba.ply) can be opened e.g. in the free program CloudCompare.
"""

import argparse

from analysis import analyze
from scanner import cmd_calibrate, cmd_scan
from simulator import cmd_simulate

# ======================================================================
# Settings (defaults, most can be changed from the command line)
# ======================================================================
DEFAULTS = dict(
    tilt_horizon=60.0,   # tilt servo angle at which the LiDAR is level
    tilt_dir=1.0,        # +1: larger tilt angle = beam points up, -1 the opposite
    axis_height=0.06,    # height of the tilt axis above the pan axis (m)
    lidar_offset=0.02,   # distance from the tilt axis to the LiDAR front face (m)
    min_range=0.20,      # closest reliable TF-Luna distance (m)
    max_range=8.0,       # farthest TF-Luna distance (m)
    min_amp=100,         # weaker reflections are discarded
    pan_min=0, pan_max=180, pan_step=2,
    tilt_min=0, tilt_max=180, tilt_step=2,
)


# ======================================================================
# Command line
# ======================================================================
def main():
    p = argparse.ArgumentParser(description="Toa ruumala mõõtmine LiDAR-skanneriga")
    p.add_argument("--tilt-horizon", type=float, default=DEFAULTS["tilt_horizon"],
                   help="tilt-servo nurk, mille juures LiDAR on loodis")
    p.add_argument("--tilt-dir", type=float, default=DEFAULTS["tilt_dir"],
                   help="+1 või -1, kui tilt pöörab vastupidi")
    p.add_argument("--samm", type=int, default=2, help="skaneerimise samm kraadides")
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("skanni", help="skaneeri üks asukoht (pooled A ja B)")
    s.add_argument("--port", required=True, help="nt COM3 või /dev/ttyUSB0")
    s.add_argument("--nimi", required=True, help="nt asukoht1")
    s.add_argument("--kaust", default="skaneeringud")

    c = sub.add_parser("kalibreeri", help="leia tilt-servo loodinurk")
    c.add_argument("--port", required=True)

    a = sub.add_parser("arvuta", help="ühenda skaneeringud ja arvuta ruumala")
    a.add_argument("kaust", nargs="?", default="skaneeringud")
    a.add_argument("--voksel", type=float, default=0.04, help="vokslisuurus meetrites")

    m = sub.add_parser("simuleeri", help="tee testandmed simuleeritud toast")
    m.add_argument("--kaust", default="sim_skaneeringud")
    m.add_argument("--asukohti", type=int, default=3, choices=[1, 2, 3, 4])
    m.add_argument("--seed", type=int, default=1)

    args = p.parse_args()
    cfg = dict(DEFAULTS)
    cfg["tilt_horizon"] = args.tilt_horizon
    cfg["tilt_dir"] = args.tilt_dir
    cfg["pan_step"] = cfg["tilt_step"] = args.samm

    if args.cmd == "skanni":
        cmd_scan(args, cfg)
    elif args.cmd == "kalibreeri":
        cmd_calibrate(args, cfg)
    elif args.cmd == "arvuta":
        analyze(args.kaust, cfg, vox=args.voksel)
    elif args.cmd == "simuleeri":
        cmd_simulate(args, cfg)


if __name__ == "__main__":
    main()
