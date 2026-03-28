"""
follow.py — Experiment 3: Manual Sequence Recorder and Playback
================================================================

WHAT THIS DOES
--------------
Lets you teach the arm a sequence of positions by hand:
  1. Manually move the arm to each position (servos released = arm goes limp)
  2. Press S to save that position
  3. Repeat as many times as needed
  4. Name the file and add a description
  5. Later, play back the exact sequence from the saved file

Use this for any repeatable task that doesn't depend on vision —
for example a fixed conveyor transfer, a reset routine, or a demo sequence.

MENU OPTIONS
------------
  1 = Record new sequence
  2 = Play existing sequence
  3 = List saved sequences
  q = Quit

RECORDING COMMANDS
------------------
  r = Release all servos (arm goes limp — move it by hand)
  s = Read and save current position (locks servos first)
  u = Undo last saved position
  v = View all saved positions so far
  d = Done — name the file, add a description, save to disk
  q = Quit without saving

IMPORTANT SAFETY NOTE
---------------------
When you press R, the arm goes limp immediately. If the arm is in an
extended position it may fall. Always hold the arm before pressing R.

HOW POSITIONS ARE SAVED
------------------------
Each sequence is saved as a JSON file in the 'sequences/' folder.
The file stores the name, description, and all step angles.
You can manually edit the JSON to change speed or delay per step.

JSON FORMAT
-----------
{
  "name": "pick_red",
  "message": "Pick red object from Zone A and drop in box",
  "steps": [
    {"step": 1, "angles": [0.5, 30.2, -29.8, -0.4, 90.0, 0.0], "speed": 30, "delay": 1.0},
    {"step": 2, "angles": [45.0, 60.0, -50.0, -10.0, 75.0, 0.0], "speed": 30, "delay": 1.5}
  ]
}

TWEAKING
--------
speed (per step in JSON):
    Movement speed for that step (0–100). Default 30. Reduce for delicate moves.

delay (per step in JSON):
    Seconds to pause after reaching that step. Default 1.0.
    Increase if the arm needs more time to settle before the next move.

HOME (top of this file):
    The rest position the arm returns to after playback.

SAVES_DIR:
    Folder where JSON files are stored. Default 'sequences/'.
    Change this if you want files in a different location.
"""

from pymycobot import MechArm
import time
import json
import os
import glob

# ═══════════════════════════════════════════════════════════════════════════════
# CONFIG
# ═══════════════════════════════════════════════════════════════════════════════

PORT      = '/dev/ttyAMA0'
BAUD      = 1000000
HOME      = [0, 0, 0, 0, 90, 0]    # arm rest position
SAVES_DIR = 'sequences'             # folder for saved JSON files

# ═══════════════════════════════════════════════════════════════════════════════
# END OF CONFIG
# ═══════════════════════════════════════════════════════════════════════════════

print("Connecting to MechArm...")
mc = MechArm(PORT, BAUD)
time.sleep(1)

os.makedirs(SAVES_DIR, exist_ok=True)


# ─────────────────────────────────────────────────────────────────────────────
# HELPERS
# ─────────────────────────────────────────────────────────────────────────────

def wait_for_arm(timeout=8):
    """Block until all joints stop moving."""
    prev = None
    for _ in range(timeout * 2):
        time.sleep(0.5)
        curr = mc.get_angles()
        if curr and prev:
            if all(abs(curr[i] - prev[i]) < 0.5 for i in range(6)):
                return curr
        prev = curr
    return prev


def get_angles_safe(retries=8, delay=0.4):
    """
    Read joint angles with retry.

    Returns None if the arm serial read fails repeatedly.
    Filters out all-zero results which indicate a failed read.
    """
    for attempt in range(retries):
        angles = mc.get_angles()
        if angles and len(angles) == 6 and any(a != 0 for a in angles):
            return angles
        print(f"  Retry {attempt+1}/{retries}...")
        time.sleep(delay)
    return None


def lock_in_place(angles):
    """
    Re-engage all 6 servos at the current position without moving the arm.

    Sends the already-read angles back at speed=5 (minimum), which
    activates servo torque without causing any noticeable movement.
    """
    mc.send_angles(angles, 5)
    time.sleep(1.2)


def release_all():
    """Release all 6 servos so the arm can be moved freely by hand."""
    mc.release_all_servos()
    time.sleep(0.3)
    print("  Servos released — move arm freely. Hold it first!")


def list_sequence_files():
    """Return sorted list of all .json files in SAVES_DIR."""
    return sorted(glob.glob(os.path.join(SAVES_DIR, '*.json')))


# ─────────────────────────────────────────────────────────────────────────────
# RECORDER
# ─────────────────────────────────────────────────────────────────────────────

def record_mode():
    """
    Interactive recording session.

    The user moves the arm to each desired position and presses S.
    All positions are collected in memory. When done (D), the user
    names the file and adds a description before saving to disk.
    """
    recorded = []

    print("\n" + "═"*50)
    print("RECORD MODE")
    print("═"*50)
    print("Commands:")
    print("  r = release servos (arm goes limp — hold it first!)")
    print("  s = save current position")
    print("  u = undo last saved position")
    print("  v = view all saved positions")
    print("  d = done — save file")
    print("  q = quit without saving")
    print()

    while True:
        cmd = input(f"[{len(recorded)} saved] Command: ").strip().lower()

        if cmd == 'r':
            release_all()

        elif cmd == 's':
            # Read FIRST before doing anything else.
            # Any arm movement between read and lock would give wrong angles.
            print("  Reading position...")
            angles = get_angles_safe()

            if not angles:
                print("  ERROR: Could not read angles.")
                print("  Try pressing 'r', repositioning, then 's' again.")
                continue

            # Lock servos by re-sending these exact angles at minimum speed.
            # This does not move the arm — it just re-engages torque.
            print("  Locking servos in place...")
            lock_in_place(angles)

            step  = len(recorded) + 1
            entry = {
                'step':   step,
                'angles': angles,
                'speed':  30,    # edit in JSON for custom speed per step
                'delay':  1.0,   # edit in JSON for custom pause per step
            }
            recorded.append(entry)
            print(f"  Position {step} saved: {[round(a, 2) for a in angles]}")

        elif cmd == 'u':
            if recorded:
                removed = recorded.pop()
                print(f"  Removed position {removed['step']}: "
                      f"{[round(a, 2) for a in removed['angles']]}")
                # Renumber remaining steps
                for i, e in enumerate(recorded):
                    e['step'] = i + 1
            else:
                print("  Nothing to undo.")

        elif cmd == 'v':
            if not recorded:
                print("  No positions saved yet.")
            else:
                print(f"\n  {'Step':<6} {'Angles':<65} {'Speed':<7} Delay")
                print("  " + "─" * 88)
                for e in recorded:
                    a = [round(x, 2) for x in e['angles']]
                    print(f"  {e['step']:<6} {str(a):<65} {e['speed']:<7} {e['delay']}s")
                print()

        elif cmd == 'd':
            if not recorded:
                print("  No positions to save. Add at least one first.")
                continue

            # Get filename
            while True:
                name = input("  File name (no spaces, no extension): ").strip()
                if not name:
                    print("  Name cannot be empty.")
                    continue
                name     = name.replace(' ', '_')
                filepath = os.path.join(SAVES_DIR, f"{name}.json")
                if os.path.exists(filepath):
                    ow = input(f"  '{filepath}' exists. Overwrite? (y/n): ").strip().lower()
                    if ow != 'y':
                        continue
                break

            # Get description
            message = input("  Description for this sequence: ").strip()
            if not message:
                message = "(no description)"

            data = {
                'name':    name,
                'message': message,
                'steps':   recorded,
            }
            with open(filepath, 'w') as f:
                json.dump(data, f, indent=2)

            print(f"\n  Saved {len(recorded)} positions to '{filepath}'")
            print(f"  Description: {message}")
            return

        elif cmd == 'q':
            print("  Exiting without saving.")
            return

        else:
            print("  Unknown command. Use: r / s / u / v / d / q")


# ─────────────────────────────────────────────────────────────────────────────
# PLAYER
# ─────────────────────────────────────────────────────────────────────────────

def play_mode():
    """
    List all saved sequences, let the user pick one, then play it back.

    The arm always homes first, executes all steps in order (respecting
    per-step speed and delay from the JSON), then returns home.
    """
    files = list_sequence_files()

    if not files:
        print("\n  No sequence files found in 'sequences/' folder.")
        print("  Use option 1 to record a sequence first.")
        return

    print("\n" + "═"*50)
    print("SELECT SEQUENCE TO PLAY")
    print("═"*50)
    print(f"\n  {'#':<5} {'File name':<30} {'Steps':<7} Description")
    print("  " + "─" * 75)

    loaded = []
    for i, filepath in enumerate(files):
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
            name    = data.get('name',    os.path.basename(filepath))
            message = data.get('message', '(no description)')
            steps   = data.get('steps',   [])
            loaded.append((filepath, data, steps))
            print(f"  {i+1:<5} {name:<30} {len(steps):<7} {message}")
        except Exception as e:
            print(f"  {i+1:<5} {os.path.basename(filepath):<30} {'?':<7} [ERROR: {e}]")
            loaded.append((filepath, None, None))

    print()

    while True:
        raw = input(f"  Select (1–{len(files)}) or q to go back: ").strip().lower()
        if raw == 'q':
            return
        if raw.isdigit() and 1 <= int(raw) <= len(files):
            idx = int(raw) - 1
            break
        print(f"  Enter a number between 1 and {len(files)}.")

    filepath, data, steps = loaded[idx]

    if data is None or steps is None:
        print("  Cannot play — file has errors.")
        return

    print(f"\n  Sequence : {data.get('name')}")
    print(f"  Desc     : {data.get('message')}")
    print(f"  Steps    : {len(steps)}\n")

    confirm = input("  Start playback? (y/n): ").strip().lower()
    if confirm != 'y':
        print("  Cancelled.")
        return

    # Home first
    print("\n  Moving to home position...")
    mc.send_angles(HOME, 30)
    wait_for_arm()
    print("  At home.\n")
    time.sleep(1)

    # Execute each step
    for entry in steps:
        step   = entry['step']
        angles = entry['angles']
        speed  = entry.get('speed', 30)
        delay  = entry.get('delay', 1.0)

        print(f"  Step {step}/{len(steps)}: "
              f"{[round(a, 2) for a in angles]}  "
              f"speed={speed}  delay={delay}s")

        mc.send_angles(angles, speed)
        wait_for_arm()
        time.sleep(delay)
        print("    Done.")

    print(f"\n  Sequence complete.")
    print("  Returning to home...")
    mc.send_angles(HOME, 30)
    wait_for_arm()
    print("  Done.\n")


# ─────────────────────────────────────────────────────────────────────────────
# MAIN MENU
# ─────────────────────────────────────────────────────────────────────────────

def main():
    print("\n" + "═"*40)
    print("   MechArm Sequence Tool — follow.py")
    print("═"*40)

    while True:
        print("\n  1 = Record new sequence")
        print("  2 = Play existing sequence")
        print("  3 = List saved sequences")
        print("  q = Quit")
        print()

        choice = input("  Choice: ").strip().lower()

        if choice == '1':
            record_mode()

        elif choice == '2':
            play_mode()

        elif choice == '3':
            files = list_sequence_files()
            if not files:
                print("\n  No sequences saved yet.")
            else:
                print(f"\n  {'#':<5} {'File name':<30} {'Steps':<7} Description")
                print("  " + "─" * 75)
                for i, filepath in enumerate(files):
                    try:
                        with open(filepath, 'r') as f:
                            data = json.load(f)
                        name    = data.get('name',    os.path.basename(filepath))
                        message = data.get('message', '(no description)')
                        steps   = data.get('steps',   [])
                        print(f"  {i+1:<5} {name:<30} {len(steps):<7} {message}")
                    except Exception:
                        print(f"  {i+1:<5} {os.path.basename(filepath):<37} [unreadable]")

        elif choice == 'q':
            print("\n  Goodbye.")
            mc.send_angles(HOME, 20)
            time.sleep(2)
            break

        else:
            print("  Unknown choice. Enter 1, 2, 3, or q.")


if __name__ == '__main__':
    main()
