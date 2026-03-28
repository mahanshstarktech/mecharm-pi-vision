"""
calibrate.py — HSV Color Range Tuner for MechArm Pi 270 experiments.

PURPOSE
-------
Use this tool BEFORE running color_sort.py to find the correct HSV
(Hue, Saturation, Value) ranges for each of your colored objects.
Different lighting conditions change how colors appear to the camera,
so you must tune these values in your actual setup.

HOW TO USE
----------
1. Run this script:
       python calibrate.py

2. A window called "Tuner" opens showing:
   - LEFT half: live camera feed
   - RIGHT half: mask (white = detected, black = ignored)

3. Place ONE colored object in front of the camera.

4. Drag the sliders until ONLY that object is white in the right half.
   Everything else should be black.

5. Press S to print the final values to the terminal.

6. Repeat steps 3–5 for each color you need.

7. Copy the printed values into the COLOR_RANGES dict in color_sort.py.

TIPS
----
- Do this under the same lighting you will use during the actual run.
- For RED: red wraps around in HSV (exists at H=0–10 AND H=170–180).
  You will need TWO range entries for red. See color_sort.py for an example.
- If the mask is noisy (speckles), increase S lo (saturation minimum).
- If the mask misses part of the object, widen H range or lower S lo / V lo.

CONTROLS
--------
  Drag sliders  — adjust HSV range in real time
  S key         — save/print current values to terminal (copy from there)
  Q key         — quit
"""

import cv2
import numpy as np

# ── Config ────────────────────────────────────────────────────────────────────
CAMERA_IDX = 0      # change to 1 if camera 0 doesn't open
FRAME_W    = 320
FRAME_H    = 240
# ─────────────────────────────────────────────────────────────────────────────

cap = cv2.VideoCapture(CAMERA_IDX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
cap.set(cv2.CAP_PROP_FPS, 10)

if not cap.isOpened():
    raise RuntimeError(
        f"Cannot open camera {CAMERA_IDX}. "
        "Try changing CAMERA_IDX to 1 at the top of this file."
    )


def nothing(x):
    pass


cv2.namedWindow("Tuner", cv2.WINDOW_NORMAL)
cv2.createTrackbar("H lo", "Tuner", 0,   180, nothing)
cv2.createTrackbar("H hi", "Tuner", 180, 180, nothing)
cv2.createTrackbar("S lo", "Tuner", 80,  255, nothing)
cv2.createTrackbar("S hi", "Tuner", 255, 255, nothing)
cv2.createTrackbar("V lo", "Tuner", 50,  255, nothing)
cv2.createTrackbar("V hi", "Tuner", 255, 255, nothing)

frame_count = 0
print("\nHSV Tuner running.")
print("Drag sliders until only your object is white in the right half.")
print("Press S to save values, Q to quit.\n")

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    frame_count += 1
    # Process every 3rd frame to reduce CPU load on Raspberry Pi
    if frame_count % 3 != 0:
        continue

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    hl = cv2.getTrackbarPos("H lo", "Tuner")
    hh = cv2.getTrackbarPos("H hi", "Tuner")
    sl = cv2.getTrackbarPos("S lo", "Tuner")
    sh = cv2.getTrackbarPos("S hi", "Tuner")
    vl = cv2.getTrackbarPos("V lo", "Tuner")
    vh = cv2.getTrackbarPos("V hi", "Tuner")

    mask     = cv2.inRange(hsv, np.array([hl, sl, vl]), np.array([hh, sh, vh]))
    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    combined = np.hstack([frame, mask_bgr])

    # Overlay current values on the frame
    cv2.putText(combined,
                f"H:[{hl}-{hh}] S:[{sl}-{sh}] V:[{vl}-{vh}]  S=save Q=quit",
                (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 220, 255), 1)

    cv2.imshow("Tuner", combined)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):
        print("Exiting.")
        break

    elif key == ord('s'):
        print("\n" + "─" * 50)
        print("COPY THIS into COLOR_RANGES in color_sort.py:")
        print(f"    (np.array([{hl}, {sl}, {vl}]),  np.array([{hh}, {sh}, {vh}])),")
        print("─" * 50)
        print(f"Raw values: H[{hl}–{hh}]  S[{sl}–{sh}]  V[{vl}–{vh}]")
        print()

cap.release()
cv2.destroyAllWindows()
