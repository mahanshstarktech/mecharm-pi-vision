"""
utils.py — Shared utilities for all MechArm Pi 270 experiments.

This file is imported by color_sort.py, shape_sort.py, and follow.py.
It contains every function that is common across all three experiments:
  - Arm connection and homing
  - Camera initialisation
  - Arm movement helpers (wait, angle reading)
  - Scan tier builder (auto face-down J4 formula)
  - Pick-and-place core sequence

Do NOT run this file directly. Import it from the experiment scripts.
"""

import cv2
import time
from pymycobot import MechArm

# ─────────────────────────────────────────────────────────────────────────────
# DEFAULT HARDWARE CONSTANTS
# These are overridden by the constants at the top of each experiment script.
# They are here only as fallback defaults so imports don't break.
# ─────────────────────────────────────────────────────────────────────────────
DEFAULT_PORT  = '/dev/ttyAMA0'
DEFAULT_BAUD  = 1000000
DEFAULT_SPEED = 35

# ─────────────────────────────────────────────────────────────────────────────
# ARM CONNECTION
# ─────────────────────────────────────────────────────────────────────────────

def connect_arm(home_angles, speed, port=DEFAULT_PORT, baud=DEFAULT_BAUD):
    """
    Connect to MechArm, home it, and fully open the gripper.

    Parameters
    ----------
    home_angles : list[float]
        6-element list of joint angles for the home/rest position.
    speed : int
        Movement speed (0–100). 35 is a safe default.
    port : str
        Serial port the arm is connected to. Default '/dev/ttyAMA0'.
    baud : int
        Baud rate. MechArm Pi 270 uses 1,000,000.

    Returns
    -------
    MechArm
        Connected and homed arm instance.

    Raises
    ------
    Exception
        If the serial connection fails (wrong port, arm off, etc.).
    """
    mc = MechArm(port, baud)
    time.sleep(1)                          # give firmware time to boot
    mc.send_angles(home_angles, speed)
    wait_for_arm(mc)
    mc.set_gripper_value(100, 50)          # fully open gripper at startup
    time.sleep(1)
    return mc


# ─────────────────────────────────────────────────────────────────────────────
# CAMERA
# ─────────────────────────────────────────────────────────────────────────────

def open_camera(camera_idx=0, frame_w=320, frame_h=240):
    """
    Open USB camera and configure it for low-RAM operation on Raspberry Pi.

    Resolution is kept at 320×240 and FPS at 10 to avoid hangs on 2 GB Pi.
    Buffer size is set to 1 so we always get the latest frame, not a queued one.

    Parameters
    ----------
    camera_idx : int
        OpenCV camera index. Usually 0 for the first USB camera.
    frame_w : int
        Frame width in pixels.
    frame_h : int
        Frame height in pixels.

    Returns
    -------
    cv2.VideoCapture
        Opened camera object.

    Raises
    ------
    RuntimeError
        If the camera cannot be opened.
    """
    cap = cv2.VideoCapture(camera_idx)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  frame_w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_h)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)    # critical — prevents RAM fill-up
    cap.set(cv2.CAP_PROP_FPS, 10)
    if not cap.isOpened():
        raise RuntimeError(
            f"Cannot open camera index {camera_idx}. "
            "Check USB connection and try index 1 if 0 fails."
        )
    return cap


def grab_fresh_frame(cap):
    """
    Flush the camera buffer and return the latest frame.

    Calling cap.read() alone returns a buffered (possibly stale) frame.
    Calling grab() three times discards the queue, then retrieve() gets the newest.

    Parameters
    ----------
    cap : cv2.VideoCapture

    Returns
    -------
    numpy.ndarray or None
        BGR frame, or None if capture failed.
    """
    for _ in range(3):
        cap.grab()
    ret, frame = cap.retrieve()
    return frame if ret else None


# ─────────────────────────────────────────────────────────────────────────────
# ARM MOVEMENT HELPERS
# ─────────────────────────────────────────────────────────────────────────────

def wait_for_arm(mc, timeout=6):
    """
    Block until all joints stop moving (angle delta < 0.5° between samples).

    Uses polling instead of a fixed sleep so faster moves return sooner
    and slow moves don't time out prematurely.

    Parameters
    ----------
    mc : MechArm
    timeout : int
        Maximum wait time in seconds.

    Returns
    -------
    list[float] or None
        Final joint angles when settled, or None on timeout.
    """
    prev = None
    for _ in range(timeout * 2):           # check every 0.5 s
        time.sleep(0.5)
        curr = mc.get_angles()
        if curr and prev:
            if all(abs(curr[i] - prev[i]) < 0.5 for i in range(6)):
                return curr
        prev = curr
    return prev


def get_angles_safe(mc, retries=8, delay=0.4):
    """
    Reliably read all 6 joint angles, retrying if the serial read fails.

    The arm's serial buffer sometimes has stale bytes right after a move,
    causing get_angles() to return None or all-zeros. This retries until
    we get a valid 6-element non-zero result.

    Parameters
    ----------
    mc : MechArm
    retries : int
        Maximum number of attempts.
    delay : float
        Seconds to wait between attempts.

    Returns
    -------
    list[float] or None
        6-element angle list, or None if all retries fail.
    """
    for _ in range(retries):
        angles = mc.get_angles()
        if angles and len(angles) == 6 and any(a != 0 for a in angles):
            return angles
        time.sleep(delay)
    return None


# ─────────────────────────────────────────────────────────────────────────────
# SCAN TIER BUILDER
# ─────────────────────────────────────────────────────────────────────────────

def keep_face_down(j1, j2, j3, j5=90):
    """
    Compute J4 so the wrist/camera always points straight down.

    The MechArm wrist (J4) must compensate for the combined shoulder+elbow
    angle to keep the camera perpendicular to the ground. The formula is:
        J4 = -(J2 + J3)

    J5 (wrist roll) is passed in per tier — it decreases as the arm
    extends outward to maintain the downward camera angle at range.

    Parameters
    ----------
    j1 : float   Base rotation angle (degrees).
    j2 : float   Shoulder pitch angle (degrees).
    j3 : float   Elbow angle (degrees).
    j5 : float   Wrist roll angle (degrees). Default 90.

    Returns
    -------
    list[float]
        Full 6-element angle list [J1, J2, J3, J4, J5, J6].
    """
    j4 = -(j2 + j3)
    j4 = max(-180, min(180, j4))          # clamp within hardware limits
    return [j1, j2, j3, j4, j5, 0]


def build_scan_tiers(scan_tiers_j123):
    """
    Convert a list of [J1, J2, J3, J5] entries into full 6-angle poses.

    Each tier represents a different reach distance from the arm base.
    J4 is automatically calculated using keep_face_down().

    Parameters
    ----------
    scan_tiers_j123 : list[list]
        Each inner list is [J1, J2, J3, J5].

    Returns
    -------
    list[list[float]]
        List of full 6-angle tier poses.
    """
    tiers = []
    for j1, j2, j3, j5 in scan_tiers_j123:
        tiers.append(keep_face_down(j1, j2, j3, j5))
    return tiers


# ─────────────────────────────────────────────────────────────────────────────
# PICK AND PLACE CORE
# ─────────────────────────────────────────────────────────────────────────────

def pick_and_place(mc, location_config, box_angles, home_angles, speed):
    """
    Execute the full pick-and-place sequence for one object.

    Sequence:
      1. Move to pick_above (safe hover position above the object zone)
      2. Fully open gripper (value=100) to clear the object
      3. Lower to pick_down (gripper at object level)
      4. Close gripper to grab object
      5. Straight vertical lift (raises J2 only, keeping J1 fixed)
      6. Move to pick_above (safe height before swinging)
      7. Swing to box position
      8. Release gripper
      9. Return home

    Parameters
    ----------
    mc : MechArm
    location_config : dict
        Must contain keys:
          'pick_above'    : list[float] — 6 angles for hover position
          'pick_down'     : list[float] — 6 angles at object level
          'gripper_close' : int         — gripper close value (0–100)
    box_angles : list[float]
        6 angles for the drop-off box position.
    home_angles : list[float]
        6 angles for the home/rest position.
    speed : int
        Movement speed (0–100).

    Returns
    -------
    bool
        True on success, False if angles could not be read for the lift.
    """
    loc = location_config

    # 1. Move above pick zone
    print("  [MOVE] Moving to pick_above...")
    mc.send_angles(loc['pick_above'], speed)
    wait_for_arm(mc)

    # 2. Fully open gripper before descending
    print("  [GRIPPER] Fully opening...")
    mc.set_gripper_value(100, 50)
    time.sleep(1.5)                        # gripper takes ~1.5 s to fully open

    # 3. Lower to object
    print("  [MOVE] Lowering to pick_down...")
    mc.send_angles(loc['pick_down'], speed)
    wait_for_arm(mc)
    time.sleep(0.5)                        # settle before gripping

    # 4. Close gripper
    print(f"  [GRIPPER] Closing to {loc['gripper_close']}...")
    mc.set_gripper_value(loc['gripper_close'], 80)
    time.sleep(2.0)                        # gripper takes ~2 s to fully close

    # 5. Straight vertical lift
    # Read current angles and raise only J2 (shoulder) so the lift path
    # is perpendicular — the object goes straight up, not in an arc.
    print("  [MOVE] Straight vertical lift...")
    current = get_angles_safe(mc)
    if current and len(current) == 6:
        lift_angles    = current[:]
        lift_angles[1] -= 20               # raise J2 by 20° — tune if needed
        lift_angles[3]  = -(lift_angles[1] + lift_angles[2])   # keep face down
        lift_angles[3]  = max(-180, min(180, lift_angles[3]))
        mc.send_angles(lift_angles, speed)
        wait_for_arm(mc)
        time.sleep(0.3)
    else:
        print("  [WARN] Could not read angles for lift — skipping to pick_above")

    # 6. Safe height before swinging
    print("  [MOVE] Moving to pick_above...")
    mc.send_angles(loc['pick_above'], speed)
    wait_for_arm(mc)
    time.sleep(0.5)

    # 7. Swing to box
    print("  [MOVE] Moving to box...")
    mc.send_angles(box_angles, speed)
    wait_for_arm(mc)

    # 8. Release
    print("  [GRIPPER] Releasing...")
    mc.set_gripper_value(100, 50)
    time.sleep(1.0)

    # 9. Return home
    print("  [MOVE] Returning home...")
    mc.send_angles(home_angles, speed)
    wait_for_arm(mc)

    return True


# ─────────────────────────────────────────────────────────────────────────────
# J1 SWEEP SCAN
# ─────────────────────────────────────────────────────────────────────────────

def j1_sweep(mc, cap, tier_angles, detect_fn, target, scan_speed,
             j1_start, j1_end, j1_step):
    """
    Move to a tier pose and sweep J1 (base rotation) looking for a target.

    At each J1 step, one camera frame is captured and passed to detect_fn.
    If detect_fn returns a non-None result, the sweep stops immediately.

    Parameters
    ----------
    mc : MechArm
    cap : cv2.VideoCapture
    tier_angles : list[float]
        6-angle pose for this scan tier (from build_scan_tiers).
    detect_fn : callable
        Function with signature detect_fn(cap, target) → (detection, frame).
        Should return (None, frame) if target not found, (dict, frame) if found.
    target : str
        Color name or shape name being searched for.
    scan_speed : int
        J1 rotation speed (0–100). Keep low (15–20) for reliable detection.
    j1_start : int
        Starting J1 angle in degrees.
    j1_end : int
        Ending J1 angle in degrees.
    j1_step : int
        Degrees to advance J1 each step.

    Returns
    -------
    dict or None
        Detection dict from detect_fn with added keys 'resume_j1' and
        'tier_angles', or None if target not found in full sweep.
    """
    pose    = tier_angles[:]
    pose[0] = j1_start
    print(f"  [SCAN] Tier pose: {[round(a, 1) for a in pose]}")
    mc.send_angles(pose, 35)
    wait_for_arm(mc)

    print(f"  [SCAN] Sweeping J1 {j1_start}°→{j1_end}° "
          f"looking for {target.upper()}")

    for j1 in range(j1_start, j1_end + 1, j1_step):
        mc.send_angle(1, j1, scan_speed)
        time.sleep(0.5)                    # settle before capture

        detection, frame = detect_fn(cap, target)

        if frame is not None:
            info = (f"TARGET={target.upper()}  J1={j1}  "
                    f"{'HIT!' if detection else 'scanning...'}")
            cv2.putText(frame, info, (5, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 255), 1)
            cv2.imshow("MechArm Vision", frame)
            cv2.waitKey(1)

        if detection:
            print(f"  [SCAN] HIT {target.upper()}  "
                  f"area={detection.get('area', '?'):.0f}  J1={j1}°")
            detection['resume_j1']   = j1
            detection['tier_angles'] = tier_angles
            return detection

    print(f"  [SCAN] {target.upper()} not found in this tier.")
    return None


# ─────────────────────────────────────────────────────────────────────────────
# PRIORITY QUEUE
# ─────────────────────────────────────────────────────────────────────────────

def ask_priority(known_targets, label='colors'):
    """
    Ask the user to enter a pick priority order at the terminal.

    The user types target names space-separated. The same target can
    appear multiple times (e.g. 'red blue red green'). Pressing Enter
    with no input uses the default order (all targets once each).

    Parameters
    ----------
    known_targets : list[str]
        Valid target names (colors or shapes).
    label : str
        Display label — 'colors' or 'shapes'.

    Returns
    -------
    list[str]
        Ordered list of target names to pick.
    """
    print(f"\nAvailable {label}: {known_targets}")
    print(f"Enter pick priority order, space separated.")
    print(f"Same {label[:-1]} can repeat. Example: {' '.join(known_targets[:2])} "
          f"{known_targets[0]}")
    print("Press Enter with no input to use default order.\n")

    while True:
        raw = input("Pick order: ").strip().lower()
        if not raw:
            queue = known_targets[:]
            print(f"Using default order: {queue}")
            return queue
        tokens = raw.split()
        invalid = [t for t in tokens if t not in known_targets]
        if invalid:
            print(f"Unknown {label}: {invalid}. Try again.")
            continue
        print(f"Pick queue: {tokens}")
        return tokens
