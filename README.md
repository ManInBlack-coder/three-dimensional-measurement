# Room volume with a LiDAR scanner (even with furniture in the room)

An ESP32 turns a LiDAR with two servos and measures the room in every direction. The room is scanned from several positions, and a program on the computer merges the scans, finds the floor, ceiling and walls, and calculates:

- **room volume**: the space between the walls, floor and ceiling, regardless of furniture
- **free air volume**
- **volume of furniture and hidden areas**

You don't need to measure where the scanner stood: the program works this out from the shape of the walls.

## How the measurement works, in simple terms

**1. One beam gives one point.** The TF-Luna is a laser rangefinder: it measures how far away the thing it points at is. The servos turn it left–right (pan) and up–down (tilt), so the program always knows which way the beam went. Direction + distance = one point on a wall, the floor, the ceiling or a piece of furniture. Repeat this every 2° in all directions and you get a cloud of about 16,000 points per position.

**2. Several positions are merged into one.** From one spot you can't see behind furniture, so the room is scanned from 3–4 places. Each scan comes out in its own coordinates, because the program doesn't know where you put the scanner. It turns and shifts the scans until their walls line up, like laying sheets of tracing paper on top of each other until the drawings match. Only the top part of the walls, near the ceiling, is used for this, because furniture rarely reaches that high.

**3. Room volume = floor area × height.** The floor and the ceiling are the lowest and the highest large flat surfaces, and the distance between them is the height. The floor plan is drawn from the wall points just under the ceiling, above the furniture, so the walls are known even where a wardrobe stands in front of them.

**4. A beam proves that the air it passed through is empty.** On its way to the point it hits, the beam flies through empty air. So the space between the scanner and every measured point is known to be free. The program divides the room into 4 cm cubes. A cube that some beam passed through is air. Everything else is either furniture or space that no scanner could see.

```
+----------------------------------------------+
|                 *              *             |
|              .           .                   |
|           .        .                         |
|        .     .                          ? ?  |
|     .  .                +---------+ ? ? ? ?  |
|  S . . . . . . . . . . *| cabinet | ? ? ? ?  |
|     .  .                +---------+ ? ? ? ?  |
|        .     .                          ? ?  |
|           .        .                         |
|              .           .                   |
|                 *              *             |
+----------------------------------------------+

S  scanner (top view)
.  a beam passed here    -> known to be air
*  the beam hit a surface -> one point in the cloud
?  no beam reached here  -> counted as "furniture or hidden"
```

This is why more positions help: a scanner placed on the other side of the cabinet sees the `?` area, and that space moves from "hidden" to "air".

### Example result

A simulated 4.6 × 3.7 × 2.55 m room with four pieces of furniture, scanned from 3 positions ([`examp/`](examp/)):

![Floor plan and point cloud of the simulated room](examp/simuleeritud_tuba.png)

- **Left, floor plan from above.** Blue is the floor area. Black dots are the wall points near the ceiling. Orange is furniture or hidden space (darker = taller). Red ▲ marks the scanner positions. The notch in the top-right corner is a column, and it correctly counts as part of the wall, not as furniture.
- **Right, the merged point cloud.** The ceiling is hidden so you can see inside.

| | Actual | Measured ([`simuleeritud_tulemus.txt`](examp/simuleeritud_tulemus.txt)) |
|---|---|---|
| Room volume | 42.33 m³ | 41.93 m³ |
| Free air | 38.30 m³ | 37.80 m³ |
| Furniture + hidden | 4.03 m³ | 4.13 m³ |

Technical details are under [How it works (technical)](#how-it-works-technical).

## Folders

```
three-dim/
├── esp32/lidar_skanner/lidar_skanner.ino   ESP32 firmware
├── src/
│   ├── ruumala.py                          RUN: all commands (scan, calibrate, compute, simulate)
│   ├── benchmark.py                        RUN: accuracy over many simulated scans
│   ├── processing/                         measurements -> result
│   │   ├── geometry.py                     raw data -> 3D points, levelling, floor/ceiling, helpers
│   │   ├── registration.py                 aligning scans from different positions
│   │   └── analysis.py                     floor plan, volumes, output files
│   ├── hardware/
│   │   └── scanner.py                      serial link to the ESP32
│   ├── simulation/
│   │   └── simulator.py                    test data without hardware (+ optional servo/LiDAR errors)
│   └── requirements.txt
└── examp/                                  example result from a simulated room
```

## Hardware (about €40–50)

| Part | Note |
|---|---|
| ESP32 dev board | |
| TF-Luna LiDAR | 0.2–8 m, UART |
| 2× servo (MG996R or DS3218) + pan-tilt bracket | metal-gear servos are more accurate |
| 5 V / 2–3 A power supply for the servos | **don't power the servos from the ESP32** |
| Light tripod or tall stand | scanner at ~1–1.5 m height |
| Spirit level (bubble level) | the scanner must be level |

### Wiring

| Device | ESP32 |
|---|---|
| TF-Luna 5V / GND | 5V / GND |
| TF-Luna RXD (pin 2) | GPIO 17 |
| TF-Luna TXD (pin 3) | GPIO 16 |
| Pan servo signal | GPIO 13 |
| Tilt servo signal | GPIO 14 |
| Servo power | external 5 V, **also connect its negative to ESP32 GND** |

### Mechanics

- The pan servo rotates horizontally 0–180°, the tilt servo vertically 0–180°.
- Mount the LiDAR so that it points straight ahead (level) when the tilt servo is at **60°**. The beam then sweeps from 60° below the horizon, over the top, to 30° behind, which gives the two halves some overlap. It doesn't have to be exact: `kalibreeri` measures the real level angle (see Usage).
- If a larger tilt angle moves the beam down instead of up, use `--tilt-dir -1`.

## Installation

1. Arduino IDE: install the **ESP32Servo** library and upload `lidar_skanner.ino` to the board.
2. On the computer: `pip install -r src/requirements.txt`

## Usage

```
cd src

# 0) try it first without hardware (simuleeri = simulate, arvuta = compute)
python ruumala.py simuleeri
python ruumala.py arvuta sim_skaneeringud
python benchmark.py          # accuracy over many simulated scans, with servo/LiDAR errors (~5 min)

# 1) calibrate (once): scanner level, ~1 m in front of a flat empty wall, LiDAR facing it
python ruumala.py kalibreeri --port COM3

# 2) scan at each position (halves A and B; rotate the base 180° by hand in between)
python ruumala.py skanni --port COM3 --nimi asukoht1
python ruumala.py skanni --port COM3 --nimi asukoht2
python ruumala.py skanni --port COM3 --nimi asukoht3

# 3) compute, with the three values that kalibreeri printed
python ruumala.py --tilt-horizon 61.2 --pan-skaala 1.021 --tilt-skaala 0.987 arvuta skaneeringud
```

Calibration sweeps both servos across the wall. A flat wall's distance follows `r = d / cos(angle)`, so fitting that curve gives the exact level angle and how many real degrees each servo turns per commanded degree. Cheap servos are often a few % off, and in the simulator an uncalibrated 3 % error alone made the result 9 % too small.

Results are written to the `skaneeringud/` folder:
- `tulemus.txt`: volumes and the registration log
- `tuba.png`: floor plan (furniture in orange) and 3D point cloud
- `tuba.ply`: point cloud that can be opened e.g. in the free program CloudCompare

On Linux the port is usually `/dev/ttyUSB0`. Scanning one half at 2° steps takes about 5–6 minutes. `--samm 3` (step) makes it about twice as fast, but slightly less accurate.

## Tips

- **3–4 positions** give a good result. Pick them so that you can see behind every piece of furniture from at least one position.
- From each new position, **at least two walls** must be visible that were also visible from the earlier positions.
- Rotate the base **in place** (mark its spot on the floor with tape), because the two halves must share the same axis.
- Close the doors and don't move in front of the scanner while it is scanning.
- Mirrors and windows create false points. The program tolerates small holes, but large mirrors are better covered.
- A `<- KONTROLLI!` ("check!") mark in the log means the alignment of some position was uncertain. Look at `tuba.png`: if the walls are doubled, rescan that position.

## How it works (technical)

0. **Calibration**: least-squares fit of `r = d / cos(s·(angle − c))` to a pan sweep and a tilt sweep across a flat wall gives the level angle `c` and each servo's scale `s`.
1. **Points**: `x = r·cos φ·cos θ`, `y = r·cos φ·sin θ`, `z = r·sin φ` (θ = pan × pan scale, φ = (tilt − level angle) × tilt scale). Before that, range noise is cut about 2× with a 3×3 median over neighbouring beams (only where the median is within 20 cm, so edges stay sharp), and **flying points** are dropped: where the ~2° beam half-hits an edge, the sensor averages the near and far surface and the point floats in between. Such points form a "veil" along the beams, so points whose surface is seen at more than 75° from head-on are removed.
2. **Levelling**: in each scan the ceiling plane is found with RANSAC and rotated to be exactly horizontal.
3. **Registration**: positions are added greedily. The initial guess comes from FFT correlation of a top-view wall image (all rotations in 2° steps), refined with 4-DOF point-to-plane ICP. Poses are scored by wall overlap and **contradictions**: a wall must not be in a place that some scanner saw as empty. The exact angle between halves A and B is also computed.
4. **Floor and ceiling**: the lowest and highest large horizontal surface (the top of each histogram peak, so noise doesn't push them apart).
5. **Floor plan**: wall points near the ceiling (above the furniture). Holes are closed, the interior is flood-filled starting from the scanner positions, and the area is summed up. This way the walls automatically extend behind cabinets.
6. **Room volume** = area × height.
7. **Free air**: the room is divided into 4 cm cubes. A cube is free if some scanner saw past it. The rest is furniture or hidden area.

## Test results (simulator)

Simulated room: 4.6 × 3.7 × 2.55 m (42.33 m³), with a column in one corner, a wardrobe, a sofa, a chest of drawers and a shelf (4.03 m³ of furniture). Every run the scanner is tilted by up to ~1°, the base is turned by hand with up to ±12° error, and the user calibrates first on a simulated wall. `python benchmark.py` adds real-world errors one at a time (3 positions, 5 runs each):

| Scenario | Room volume error (mean / worst) | Furniture |
|---|---|---|
| ideal (1.5 cm noise) | −1.2 % / −2.0 % | 4.15 m³ |
| noise 2 cm + 1 % of distance | −2.5 % / −3.8 % | 4.08 m³ |
| servo backlash 1° | −0.8 % / −2.0 % | 4.18 m³ |
| servo jitter 0.5° | −1.3 % / −2.7 % | 4.29 m³ |
| servo turns 3 % too far | −1.4 % / −2.4 % | 4.12 m³ |
| beam 2° wide (TF-Luna) | −1.7 % / −2.7 % | 4.06 m³ |
| all of the above | −4.0 % / −5.2 % | 3.84 m³ |

The room volume comes out slightly small because the floor area does (the height is accurate to ~1 cm). The furniture volume usually comes out slightly large, because it also includes areas that no scanner saw. The noise levels above are estimates from the TF-Luna datasheet; measure your own (e.g. repeated `READ` at a wall) to see which row applies. Mirrors, windows and dark surfaces are not simulated.
