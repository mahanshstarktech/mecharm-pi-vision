# MechArm Pi 270 — Pick and Place Experiments

A collection of three pick-and-place experiments for the **Elephant Robotics MechArm Pi 270** robotic arm, built as part of an Embedded AI & IoT Systems internship. The arm uses a wrist-mounted USB camera and runs entirely on the onboard Raspberry Pi 4B (2 GB RAM).

---

## Repository Structure

```
mecharm-pick-place/
├── utils.py          # Shared utilities — imported by all three experiments
├── calibrate.py      # HSV color tuner (run before color_sort.py)
├── get_angles.py     # Interactive joint angle reader (run before any experiment)
├── color_sort.py     # Experiment 1 — pick and place by color
├── shape_sort.py     # Experiment 2 — pick and place by shape
├── follow.py         # Experiment 3 — record and replay arm sequences
└── sequences/        # Auto-created by follow.py to store JSON sequence files
```

---

## Hardware Requirements

| Component | Details |
|---|---|
| Robotic arm | Elephant Robotics MechArm Pi 270 |
| Computer | Onboard Raspberry Pi 4B (2 GB RAM) |
| Camera | USB camera mounted on the wrist (J6 flange), pointing straight down |
| Connection | UART serial — `/dev/ttyAMA0` at 1,000,000 baud |
| Gripper | Electric gripper (value range 0–100) |

---

## Software Requirements

Install dependencies on the Raspberry Pi:

```bash
pip install pymycobot opencv-python numpy
```

Verify the serial port is enabled:

```bash
ls /dev/ttyAMA0
```

If the port is missing, enable UART in `raspi-config → Interface Options → Serial`.

---

## Quick Start

### Step 1 — Find arm positions

Run `get_angles.py` to find the joint angles for each position you need
(home, pick_above, pick_down, box). The arm goes limp so you can move it by hand.

```bash
python get_angles.py
```

The script prints 6-angle arrays — copy them into the `CONFIG` section
of whichever experiment script you are using.

### Step 2 — Tune color detection (for Experiment 1 only)

```bash
python calibrate.py
```

Hold each colored object in front of the camera. Drag the HSV sliders
until only that object is white in the right half of the window. Press S
to print the values, then copy them into `COLOR_RANGES` in `color_sort.py`.

### Step 3 — Run an experiment

```bash
python color_sort.py    # Experiment 1 — sort by color
python shape_sort.py    # Experiment 2 — sort by shape
python follow.py        # Experiment 3 — record and replay
```

---

## Experiment 1 — `color_sort.py`

Scans the workspace by rotating the base joint (J1) and looks for objects
of a specified color. When found, moves to the pre-calibrated position for
that color and picks it up.

**How it works:**
1. User types pick order at startup: `red blue red green`
2. Arm sweeps J1 from −90° to +90° at three reach distances (scan tiers)
3. On detection, arm goes to the fixed `pick_above` → opens gripper →
   lowers to `pick_down` → grips → lifts → drops in box → returns home
4. Repeats for the next item in the queue

**Key config to fill in:**

```python
# In color_sort.py

# 1. Arm positions — run get_angles.py to find these
COLOR_LOCATIONS = {
    'red': {
        'pick_above':    [...],   # arm hovering above red zone
        'pick_down':     [...],   # gripper at red object level
        'gripper_close': 15,      # grip tightness (0=closed, 100=open)
    },
    ...
}
BOX_ANGLES = [...]   # where to drop objects

# 2. Color detection ranges — run calibrate.py to find these
COLOR_RANGES = {
    'red': [
        (np.array([H_lo, S_lo, V_lo]), np.array([H_hi, S_hi, V_hi])),
        # Red needs two entries because it wraps in HSV at 0° and 180°
        (np.array([170, 150, 80]),     np.array([180, 255, 255])),
    ],
    ...
}

# 3. Scan tiers — calibrate arm poses at close/mid/far distances
SCAN_TIERS_J123 = [
    [0, 30, -30, 90],   # [J1_start, J2, J3, J5] — J4 auto-calculated
    [0, 45, -40, 75],
    [0, 55, -60, 83],
]
```

---

## Experiment 2 — `shape_sort.py`

Same flow as Experiment 1 but detects **shapes** instead of colors.
Uses contour analysis — no HSV tuning needed.

**How shape detection works:**

Each contour is measured by:
- **Circularity** = `4π × area / perimeter²` (1.0 = perfect circle)
- **Vertex count** from `cv2.approxPolyDP`

These two values are matched against ranges defined in `SHAPE_DETECTORS`.

| Shape | Vertices | Circularity |
|---|---|---|
| Circle | 8+ | 0.80–1.10 |
| Square | 4–6 | 0.70–0.95 |
| Triangle | exactly 3 | 0.40–0.75 |

**Adding a new shape:**

```python
# In shape_sort.py — SHAPE_DETECTORS
'pentagon': {
    'vertices_range':    (5, 6),
    'circularity_range': (0.75, 0.92),
    'draw_color':        (0, 255, 255),   # yellow bounding box
},
```

Then add its pick positions to `SHAPE_LOCATIONS` — same format as colors.

**Removing a shape:** Delete its entry from both `SHAPE_DETECTORS` and `SHAPE_LOCATIONS`.

**Tuning if shapes are misclassified:**

```python
EPSILON_FACTOR = 0.04   # lower = more vertices, higher = fewer
# Try 0.02–0.06 if circle is detected as square or vice versa
```

---

## Experiment 3 — `follow.py`

Lets you teach the arm a sequence of positions by hand and play them back.

**Recording:**
```
1 → Record new sequence

Commands during recording:
  r = arm goes limp (hold it first!)
  s = save current position
  u = undo last position
  v = view all saved positions
  d = name file, add description, save to disk
  q = quit without saving
```

**Playing back:**
```
2 → Play existing sequence
   → lists all saved files with descriptions
   → select one → arm homes → follows all steps → returns home
```

**Editing a sequence manually** (in the JSON file):

```json
{
  "step": 2,
  "angles": [45.0, 60.0, -50.0, -10.0, 75.0, 0.0],
  "speed": 15,    ← reduce for slow/careful moves
  "delay": 2.5    ← increase for longer pauses
}
```

---

## `utils.py` — Shared Functions Reference

All three experiment scripts import from `utils.py`. You do not run it directly.

| Function | What it does |
|---|---|
| `connect_arm(home, speed, port, baud)` | Connect, home, open gripper |
| `open_camera(idx, w, h)` | Open camera with Pi-optimised settings |
| `grab_fresh_frame(cap)` | Flush buffer and return latest frame |
| `wait_for_arm(mc, timeout)` | Block until joints stop moving |
| `get_angles_safe(mc, retries, delay)` | Read angles with retry |
| `keep_face_down(j1, j2, j3, j5)` | Compute J4 to keep camera down |
| `build_scan_tiers(scan_tiers_j123)` | Convert tier list to full 6-angle poses |
| `pick_and_place(mc, loc, box, home, speed)` | Full pick-place sequence |
| `j1_sweep(mc, cap, tier, detect_fn, target, ...)` | J1 sweep with detection callback |
| `ask_priority(known_targets, label)` | Ask user for pick order |

---

## How the Camera-Down Formula Works

As the arm extends further from the base, J2 (shoulder) and J3 (elbow)
increase. Without compensation, the wrist tilts and the camera no longer
faces down. The formula in `keep_face_down()`:

```
J4 = -(J2 + J3)
```

automatically computes the correct wrist angle. J5 also decreases per tier
(90 → 75 → 60) to handle the remaining tilt at extended positions.

---

## Troubleshooting

**Arm doesn't connect:**
- Check USB/serial cable and run `ls /dev/ttyAMA0`
- Enable UART in `raspi-config`
- Make sure the arm is powered on before running the script

**Camera doesn't open:**
- Run `ls /dev/video*` to find your camera index
- Change `CAMERA_IDX = 0` to `1` if needed

**Script hangs after 10–15 seconds:**
- This is a RAM issue on 2 GB Pi. The scripts are already optimised (320×240, 10 FPS, 1-frame buffer). If it still hangs, close other applications.

**Color not detected:**
- Re-run `calibrate.py` under the same lighting conditions as the experiment
- Increase `SCAN_MIN_AREA` if getting noise, decrease if missing real objects

**Shape misclassified:**
- Adjust `EPSILON_FACTOR` between 0.02 and 0.06
- Check that objects are well-lit with clear edges (avoid reflective surfaces)

**Arm angles read as None:**
- Normal after `release_all_servos()` — the serial buffer needs time to clear
- `get_angles_safe()` retries 8 times automatically
- If it keeps failing, increase the `delay` parameter in `get_angles_safe()`

**Gripper doesn't close on object:**
- Lower `gripper_close` value in `COLOR_LOCATIONS` / `SHAPE_LOCATIONS`
- Ensure `pick_down` position places the gripper fingers around (not above) the object

---

## Author

Mahansh — Embedded AI & IoT Systems Intern  
B.Tech Computer Science (IoT), Jaipur, Rajasthan, India
