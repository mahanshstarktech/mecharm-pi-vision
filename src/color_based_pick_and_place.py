"""
color_sort.py — Experiment 1: Color-Based Pick and Place
=========================================================

WHAT THIS DOES
--------------
The arm scans the workspace by rotating J1 (base) while searching for
objects of a user-specified color. When found, it moves to a pre-calibrated
fixed position for that color and picks it up, then drops it in a box.

SETUP STEPS (do these before running)
--------------------------------------
Step 1 — Find HSV color ranges:
    Run:  python calibrate.py
    Hold each colored object in front of the camera and tune the sliders
    until only that object appears white in the mask. Press S to print the
    values. Copy them into COLOR_RANGES below.

Step 2 — Find arm positions:
    Run:  python get_angles.py
    For each color zone, record:
      - pick_above : arm hovering above the object, gripper pointing down
      - pick_down  : gripper at the object level, ready to grip
    Also record HOME_ANGLES and BOX_ANGLES.

Step 3 — Fill in the CONFIG section below.

Step 4 — Run:
    python color_sort.py

HOW TO USE
----------
When the script starts, you are asked to type a pick order.
Example input:    red blue red green
The arm will search for red first, pick it, then blue, then red again, then green.
Pressing Enter with no input uses the default order (all colors once).

TWEAKING / CALIBRATION
-----------------------
SCAN_TIERS_J123 :
    Defines arm poses for 3 scan distances (close, mid, far).
    Format: [J1, J2, J3, J5]
    J4 is auto-computed to keep the camera facing straight down.
    J5 decreases as the arm extends (90 → 75 → 60) to maintain downward view.
    Tune J2 and J3 to match your table height.

J1_SCAN_START / J1_SCAN_END :
    The angular sweep range. -90 to +90 covers a 180° arc.
    Narrow this range if objects only appear on one side.

J1_SCAN_STEP :
    Degrees between each camera snapshot during sweep.
    Smaller = more thorough but slower. 8° is a good default.

SCAN_MIN_AREA :
    Minimum contour area in pixels² to count as a real object (not noise).
    Increase if you get false detections. Decrease if real objects are missed.

gripper_close :
    How tight to grip (0=fully closed, 100=fully open).
    15 is a firm grip. Increase if the object slips; decrease if it's crushed.

lift_angles[1] -= 20 (in utils.py pick_and_place):
    How many degrees J2 raises during the straight vertical lift.
    Increase for a taller lift. Decrease if the lift overshoots.
"""

import cv2
import numpy as np
import time
from arm_utilities import (
    connect_arm, open_camera, grab_fresh_frame,
    build_scan_tiers, pick_and_place, j1_sweep, ask_priority, wait_for_arm
)

# ═══════════════════════════════════════════════════════════════════════════════
# CONFIG — edit everything in this section to match your setup
# ═══════════════════════════════════════════════════════════════════════════════

PORT       = '/dev/ttyAMA0'
BAUD       = 1000000
CAMERA_IDX = 0            # change to 1 if your camera doesn't open
SPEED      = 35           # arm movement speed (0–100). Don't go above 50.
SCAN_SPEED = 18           # J1 sweep speed — keep slow for reliable detection

FRAME_W, FRAME_H = 320, 240
SCAN_MIN_AREA    = 300    # minimum px² to count as a real object (not noise)

# Home position — arm at rest, not in the way of anything
HOME_ANGLES = [0, 0, 0, 0, 90, 0]

# Drop-off box position — where objects are released
# ← Run get_angles.py with arm over the box to find this
BOX_ANGLES  = [90, 30, -30, 0, 90, 0]

# ── Scan tiers ──────────────────────────────────────────────────────────────
# Three distances the arm scans at: close, mid, far.
# Format: [J1_start, J2, J3, J5]
# J1 is always overridden during sweep. J4 is auto-calculated.
# Tune J2 and J3 so the arm hovers at a good height above the table.
# ← Use get_angles.py with arm in a downward-facing scan pose at each distance

SCAN_TIERS_J123 = [
    [0,  30, -30, 90],    # Tier 0 — close range  ← CALIBRATE
    [0,  45, -40, 75],    # Tier 1 — mid range    ← CALIBRATE
    [0,  55, -60, 83],    # Tier 2 — far range    ← CALIBRATE
]

# ── J1 sweep range ──────────────────────────────────────────────────────────
J1_SCAN_START = -90   # start angle (degrees). -90 = full left
J1_SCAN_END   =  90   # end angle (degrees).    90 = full right
J1_SCAN_STEP  =   8   # degrees per step. Smaller = more thorough, slower

# ── Fixed pick positions per color ──────────────────────────────────────────
# For each color, calibrate pick_above and pick_down using get_angles.py:
#   pick_above : arm hovering over the object zone, camera facing down
#   pick_down  : gripper at the exact level of the object surface
#   gripper_close : grip tightness (0=closed, 100=open). 10–20 is typical.

COLOR_LOCATIONS = {
    'red': {
        'pick_above':    [0.52, 68.46, -59.85, -7.38, 75.41, 0.17],   # ← your values
        'pick_down':     [1.05, 63.63, -61.17, -4.65, 79.1,  0.08],   # ← your values
        'gripper_close': 15,
    },
    'green': {
        'pick_above':    [-40, 30, -30, 0, 90, 0],   # ← CALIBRATE
        'pick_down':     [-40, 45, -10, 0, 90, 0],   # ← CALIBRATE
        'gripper_close': 15,
    },
    'blue': {
        'pick_above':    [40, 30, -30, 0, 90, 0],    # ← CALIBRATE
        'pick_down':     [40, 45, -10, 0, 90, 0],    # ← CALIBRATE
        'gripper_close': 15,
    },
}

# ── HSV color detection ranges ───────────────────────────────────────────────
# Find these using:  python calibrate.py
#
# Red is special — it wraps around in HSV (exists at H=0–10 AND H=170–180)
# so it needs TWO range entries. All other colors need only ONE.
#
# Format per entry: (np.array([H_lo, S_lo, V_lo]), np.array([H_hi, S_hi, V_hi]))

COLOR_RANGES = {
    'red': [
        (np.array([0,   150, 80]),  np.array([10,  255, 255])),   # red at H=0
        (np.array([170, 150, 80]),  np.array([180, 255, 255])),   # red at H=180
    ],
    'blue': [
        (np.array([87, 81, 104]),   np.array([109, 192, 176])),
    ],
    'green': [
        (np.array([40, 80, 50]),    np.array([80,  255, 255])),
    ],
}

# ═══════════════════════════════════════════════════════════════════════════════
# END OF CONFIG
# ═══════════════════════════════════════════════════════════════════════════════


def detect_color(cap, target):
    """
    Scan one camera frame for the target color using HSV masking.

    Applies morphological open+close to remove noise, finds contours,
    and returns the largest matching blob.

    Parameters
    ----------
    cap : cv2.VideoCapture
    target : str
        Color name to look for (must be a key in COLOR_RANGES).

    Returns
    -------
    tuple (dict or None, numpy.ndarray or None)
        detection dict with keys: color, area, cx, cy, bx, by, bw, bh
        annotated BGR frame
    """
    frame = grab_fresh_frame(cap)
    if frame is None:
        return None, None

    hsv    = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    best   = None

    # Search only the target color (not all colors) for speed and accuracy
    ranges = COLOR_RANGES.get(target, [])
    mask   = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for (lo, hi) in ranges:
        mask |= cv2.inRange(hsv, lo, hi)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < SCAN_MIN_AREA:
            continue
        bx, by, bw, bh = cv2.boundingRect(cnt)
        M  = cv2.moments(cnt)
        cx = int(M['m10'] / M['m00']) if M['m00'] else bx + bw // 2
        cy = int(M['m01'] / M['m00']) if M['m00'] else by + bh // 2

        c = {'red':(0,0,255), 'green':(0,200,0), 'blue':(255,100,0)}.get(
             target, (200, 200, 200))
        cv2.rectangle(frame, (bx, by), (bx+bw, by+bh), c, 2)
        cv2.putText(frame, f"{target} {area:.0f}px",
                    (bx, max(0, by-5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, c, 1)

        if best is None or area > best['area']:
            best = {'color': target, 'area': area,
                    'cx': cx, 'cy': cy,
                    'bx': bx, 'by': by, 'bw': bw, 'bh': bh}

    return best, frame


def main():
    print("Connecting to MechArm...")
    mc = connect_arm(HOME_ANGLES, SPEED, PORT, BAUD)
    print("Opening camera...")
    cap = open_camera(CAMERA_IDX, FRAME_W, FRAME_H)

    SCAN_TIERS = build_scan_tiers(SCAN_TIERS_J123)
    print("\nScan tiers (J4 auto-calculated):")
    for i, t in enumerate(SCAN_TIERS):
        print(f"  Tier {i}: {[round(a, 1) for a in t]}")

    try:
        while True:
            queue = ask_priority(list(COLOR_LOCATIONS.keys()), label='colors')
            print(f"\nStarting session. Queue: {queue}\n")

            while queue:
                target = queue[0]
                print(f"\n{'='*48}")
                print(f"LOOKING FOR: {target.upper()}")
                print(f"Remaining queue: {queue}")
                print(f"{'='*48}")

                found = False

                for tier_idx, tier_angles in enumerate(SCAN_TIERS):
                    print(f"\n-- Tier {tier_idx} --")
                    detection = j1_sweep(
                        mc, cap, tier_angles, detect_color, target,
                        SCAN_SPEED, J1_SCAN_START, J1_SCAN_END, J1_SCAN_STEP
                    )
                    if detection:
                        found = True
                        pick_and_place(
                            mc, COLOR_LOCATIONS[target],
                            BOX_ANGLES, HOME_ANGLES, SPEED
                        )
                        queue.pop(0)
                        break

                if not found:
                    print(f"\n[!] {target.upper()} not found in any tier.")
                    print("  s = skip   r = retry   q = quit session")
                    choice = input("Choice: ").strip().lower()
                    if choice == 's':
                        queue.pop(0)
                    elif choice == 'q':
                        break

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            print("\n=== Queue complete! ===")
            again = input("Start new session? (y/n): ").strip().lower()
            if again != 'y':
                break

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        mc.send_angles(HOME_ANGLES, SPEED)
        time.sleep(2)
        mc.set_gripper_value(100, 50)
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
