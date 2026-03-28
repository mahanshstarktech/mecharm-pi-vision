"""
get_angles.py — Interactive joint angle reader for MechArm Pi 270.

PURPOSE
-------
Use this tool to find the joint angles for any arm position you need —
such as pick_above, pick_down, home, or box positions used in the
experiment scripts.

HOW TO USE
----------
1. Run this script:
       python get_angles.py

2. The arm connects, homes itself, and all servos are released (arm goes limp).
   HOLD the arm before running so it doesn't drop suddenly.

3. Move the arm by hand to the desired position.

4. Press Enter — the script locks the arm and reads the angles.

5. The 6 angles are printed. Copy them into the experiment script config.

6. You are asked if you want to record another position.
   Press Y to continue, N to exit.

EXAMPLE OUTPUT
--------------
   Move arm to HOME position, then press Enter...
   Locking servos...
   Angles: [0.52, 68.46, -59.85, -7.38, 75.41, 0.17]

   Record another position? (y/n): y
   Label for next position: pick_down
   Move arm to pick_down, then press Enter...
   Locking servos...
   Angles: [1.05, 63.63, -61.17, -4.65, 79.1, 0.08]

UNDERSTANDING THE 6 ANGLES
---------------------------
   Index 0 → J1 Base rotation    (-170° to +170°)
   Index 1 → J2 Shoulder pitch   (-90°  to +90° )
   Index 2 → J3 Elbow            (-90°  to +90° )
   Index 3 → J4 Wrist pitch      (-180° to +180°)
   Index 4 → J5 Wrist roll       (-180° to +180°)
   Index 5 → J6 End-effector     (-180° to +180°)

TROUBLESHOOTING
---------------
- If angles come back as None or all zeros: wait 1–2 seconds after
  releasing, then press Enter again.
- If the arm snaps to a wrong position when locked: the read happened
  while servos were still transitioning. The script retries automatically.
"""

from pymycobot import MechArm
import time

# ── Config ────────────────────────────────────────────────────────────────────
PORT = '/dev/ttyAMA0'   # change if your Pi uses a different port
BAUD = 1000000
HOME = [0, 0, 0, 0, 90, 0]
# ─────────────────────────────────────────────────────────────────────────────

print("Connecting to MechArm...")
mc = MechArm(PORT, BAUD)
time.sleep(1)

print("Homing arm...")
mc.send_angles(HOME, 20)
time.sleep(3)
print("Home reached.\n")


def get_angles_safe(retries=8, delay=0.4):
    """Read angles with retry — returns None if all retries fail."""
    for attempt in range(retries):
        angles = mc.get_angles()
        if angles and len(angles) == 6 and any(a != 0 for a in angles):
            return angles
        print(f"  Retry {attempt+1}/{retries}...")
        time.sleep(delay)
    return None


def lock_in_place(angles):
    """Re-send the current angles at minimum speed to engage servos in place."""
    mc.send_angles(angles, 5)
    time.sleep(1.2)


print("="*50)
print("ANGLE RECORDER")
print("="*50)
print("All servos will be released. Move arm by hand, then press Enter.")
print("HOLD the arm before proceeding — it will go limp.\n")

recorded = {}

while True:
    label = input("Label for this position (e.g. pick_above_red): ").strip()
    if not label:
        label = f"position_{len(recorded)+1}"

    print(f"\nReleasing servos for '{label}'...")
    mc.release_all_servos()
    time.sleep(0.3)
    print("Arm is free. Move it to the desired position.")

    input(f"Move arm to '{label}', then press Enter...")

    print("Reading angles...")
    angles = get_angles_safe()

    if not angles:
        print("ERROR: Could not read angles after multiple retries.")
        print("The arm may still be settling. Try again.\n")
        retry = input("Retry this position? (y/n): ").strip().lower()
        if retry == 'y':
            continue
        else:
            print("Skipping this position.")
    else:
        # Lock servos at the read position
        print("Locking servos...")
        lock_in_place(angles)

        # Re-read after locking for accuracy
        final = get_angles_safe()
        if final:
            angles = final

        recorded[label] = angles
        print(f"\n  '{label}': {[round(a, 2) for a in angles]}")

    print()
    another = input("Record another position? (y/n): ").strip().lower()
    if another != 'y':
        break

# ── Print summary ─────────────────────────────────────────────────────────────
print("\n" + "="*60)
print("SUMMARY — copy the relevant lines into your experiment script:")
print("="*60)
for label, angles in recorded.items():
    rounded = [round(a, 2) for a in angles]
    print(f"  '{label}': {rounded},")
print("="*60)

# Return home
print("\nReturning to home position...")
mc.send_angles(HOME, 20)
time.sleep(3)
print("Done.")
