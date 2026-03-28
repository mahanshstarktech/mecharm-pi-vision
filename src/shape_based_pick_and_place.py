"""
shape_sort.py — Experiment 2: Shape-Based Pick and Place
=========================================================

WHAT THIS DOES
--------------
Identical flow to color_sort.py but detects objects by shape (circle,
square, triangle) instead of color. Uses contour analysis:
  - Circularity  (4π×area/perimeter²) — how round the shape is
  - Vertex count (from approxPolyDP)   — how many corners it has

SETUP STEPS (do these before running)
--------------------------------------
Step 1 — Find arm positions using get_angles.py:
    For each shape zone record pick_above and pick_down.
    Also confirm HOME_ANGLES and BOX_ANGLES.

Step 2 — Fill in SHAPE_LOCATIONS below with your calibrated angles.

Step 3 — (Optional) Test shape detection live:
    Run the script and hold objects in front of the camera.
    The live window shows bounding boxes with shape labels.
    If shapes are misclassified, tune EPSILON_FACTOR or the
    circularity/vertex ranges in SHAPE_DETECTORS.

Step 4 — Run:
    python shape_sort.py

HOW TO USE
----------
Same as color_sort.py — type a pick order when prompted.
Example:  circle square triangle circle

TWEAKING / CALIBRATION
-----------------------
SHAPE_DETECTORS dict:
    Each shape has two ranges to match:
      vertices_range    : (min, max) polygon corners after approximation
      circularity_range : (min, max) where 1.0 = perfect circle

    Shape reference values:
      Circle   — many vertices (8+),  circularity ≈ 0.85–1.0
      Square   — 4–6 vertices,        circularity ≈ 0.75–0.90
      Triangle — exactly 3 vertices,  circularity ≈ 0.45–0.75

    If a square is being detected as a circle:
      → Raise the circle circularity_range minimum (e.g. 0.80 → 0.90)

    If shapes are not detected at all:
      → Reduce SCAN_MIN_AREA or improve lighting

EPSILON_FACTOR:
    Controls how tightly the polygon is approximated.
    Range: 0.02 (tight, more vertices) to 0.06 (loose, fewer vertices)
    If circle is detected as square: increase epsilon (e.g. 0.04 → 0.06)
    If square is detected as circle: decrease epsilon (e.g. 0.04 → 0.02)

ADDING A NEW SHAPE:
    1. Add an entry to SHAPE_DETECTORS with your vertex and circularity ranges.
    2. Add an entry to SHAPE_LOCATIONS with calibrated pick angles.
    That's it — no other code changes needed.

REMOVING A SHAPE:
    Delete its entry from both SHAPE_DETECTORS and SHAPE_LOCATIONS.
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
CAMERA_IDX = 0
SPEED      = 35
SCAN_SPEED = 18

FRAME_W, FRAME_H = 320, 240
SCAN_MIN_AREA    = 300    # minimum px² to count as a real object

HOME_ANGLES = [0, 0, 0, 0, 90, 0]
BOX_ANGLES  = [90, 30, -30, 0, 90, 0]   # ← calibrate with get_angles.py

SCAN_TIERS_J123 = [
    [0,  30, -30, 90],    # Tier 0 — close range  ← CALIBRATE
    [0,  45, -40, 75],    # Tier 1 — mid range    ← CALIBRATE
    [0,  55, -60, 83],    # Tier 2 — far range    ← CALIBRATE
]

J1_SCAN_START = -90
J1_SCAN_END   =  90
J1_SCAN_STEP  =   8

# ── Shape classifier settings ────────────────────────────────────────────────
# Controls polygon approximation tightness during shape classification.
# Lower = more vertices detected. Higher = fewer vertices.
# Tune between 0.02 and 0.06 if shapes are misclassified.
EPSILON_FACTOR = 0.04

# ── Shape detector rules ─────────────────────────────────────────────────────
# To add a shape: add a new key with vertices_range, circularity_range, draw_color.
# To remove a shape: delete its entry from here AND from SHAPE_LOCATIONS.
#
# Circularity formula: 4π × area / perimeter²
#   Circle   ≈ 1.0    (perfect circle = 1.0)
#   Square   ≈ 0.785
#   Triangle ≈ 0.605

SHAPE_DETECTORS = {
    'circle': {
        'vertices_range':    (8, 999),        # smooth curve = many corners
        'circularity_range': (0.80, 1.10),    # near-perfect roundness
        'draw_color':        (255, 0, 255),   # magenta bounding box
    },
    'square': {
        'vertices_range':    (4, 6),
        'circularity_range': (0.70, 0.95),
        'draw_color':        (0, 165, 255),   # orange bounding box
    },
    'triangle': {
        'vertices_range':    (3, 3),
        'circularity_range': (0.40, 0.75),
        'draw_color':        (0, 255, 0),     # green bounding box
    },
    # ── Example: add pentagon ───────────────────────────────────────────────
    # 'pentagon': {
    #     'vertices_range':    (5, 6),
    #     'circularity_range': (0.75, 0.92),
    #     'draw_color':        (0, 255, 255),
    # },
}

# ── Fixed pick positions per shape ───────────────────────────────────────────
# Calibrate with get_angles.py. Same format as color_sort.py COLOR_LOCATIONS.
SHAPE_LOCATIONS = {
    'circle': {
        'pick_above':    [0, 30, -30, 0, 90, 0],     # ← CALIBRATE
        'pick_down':     [0, 45, -10, 0, 90, 0],     # ← CALIBRATE
        'gripper_close': 15,
    },
    'square': {
        'pick_above':    [-40, 30, -30, 0, 90, 0],   # ← CALIBRATE
        'pick_down':     [-40, 45, -10, 0, 90, 0],   # ← CALIBRATE
        'gripper_close': 15,
    },
    'triangle': {
        'pick_above':    [40, 30, -30, 0, 90, 0],    # ← CALIBRATE
        'pick_down':     [40, 45, -10, 0, 90, 0],    # ← CALIBRATE
        'gripper_close': 15,
    },
}

# ═══════════════════════════════════════════════════════════════════════════════
# END OF CONFIG
# ═══════════════════════════════════════════════════════════════════════════════


def classify_shape(contour):
    """
    Classify a contour as a named shape using circularity and vertex count.

    Parameters
    ----------
    contour : numpy.ndarray
        Single OpenCV contour.

    Returns
    -------
    str or None
        Shape name from SHAPE_DETECTORS, or None if no match.
    """
    area = cv2.contourArea(contour)
    if area < SCAN_MIN_AREA:
        return None

    perimeter = cv2.arcLength(contour, True)
    if perimeter == 0:
        return None

    circularity = (4 * np.pi * area) / (perimeter * perimeter)
    epsilon     = EPSILON_FACTOR * perimeter
    approx      = cv2.approxPolyDP(contour, epsilon, True)
    vertices    = len(approx)

    for shape_name, cfg in SHAPE_DETECTORS.items():
        v_min, v_max = cfg['vertices_range']
        c_min, c_max = cfg['circularity_range']
        if v_min <= vertices <= v_max and c_min <= circularity <= c_max:
            return shape_name

    return None   # didn't match any defined shape


def detect_shape(cap, target):
    """
    Scan one camera frame for the target shape using contour analysis.

    Preprocessing: grayscale → Gaussian blur → Canny edges → dilate.
    Then find contours and classify each with classify_shape().

    Parameters
    ----------
    cap : cv2.VideoCapture
    target : str
        Shape name to look for (must be a key in SHAPE_DETECTORS).

    Returns
    -------
    tuple (dict or None, numpy.ndarray or None)
        detection dict with keys: shape, area, cx, cy, bx, by, bw, bh
        annotated BGR frame
    """
    frame = grab_fresh_frame(cap)
    if frame is None:
        return None, None

    gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges   = cv2.Canny(blurred, 50, 150)

    # Dilate edges slightly to close small gaps in contours
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    edges  = cv2.dilate(edges, kernel, iterations=1)

    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
    best = None

    for cnt in contours:
        if cv2.contourArea(cnt) < SCAN_MIN_AREA:
            continue

        shape_name = classify_shape(cnt)
        if shape_name is None or shape_name != target:
            continue

        bx, by, bw, bh = cv2.boundingRect(cnt)
        M  = cv2.moments(cnt)
        cx = int(M['m10'] / M['m00']) if M['m00'] else bx + bw // 2
        cy = int(M['m01'] / M['m00']) if M['m00'] else by + bh // 2

        draw_color = SHAPE_DETECTORS[shape_name]['draw_color']
        cv2.drawContours(frame, [cnt], -1, draw_color, 2)
        cv2.rectangle(frame, (bx, by), (bx+bw, by+bh), draw_color, 1)
        cv2.putText(frame, f"{shape_name} {cv2.contourArea(cnt):.0f}px",
                    (bx, max(0, by-5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, draw_color, 1)
        cv2.circle(frame, (cx, cy), 4, draw_color, -1)

        area = cv2.contourArea(cnt)
        if best is None or area > best['area']:
            best = {
                'shape': shape_name, 'area': area,
                'cx': cx, 'cy': cy,
                'bx': bx, 'by': by, 'bw': bw, 'bh': bh,
            }

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
    print(f"\nActive shapes: {list(SHAPE_DETECTORS.keys())}")

    try:
        while True:
            queue = ask_priority(list(SHAPE_LOCATIONS.keys()), label='shapes')
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
                        mc, cap, tier_angles, detect_shape, target,
                        SCAN_SPEED, J1_SCAN_START, J1_SCAN_END, J1_SCAN_STEP
                    )
                    if detection:
                        found = True
                        pick_and_place(
                            mc, SHAPE_LOCATIONS[target],
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
