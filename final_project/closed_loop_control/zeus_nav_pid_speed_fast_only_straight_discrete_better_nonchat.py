import numpy as np
import socket, time, sys
from aruco_localization_fast import get_pose
import cv2
import csv

coord_log = []

# OS-portable non-blocking keyboard check
def key_pressed():
    try:
        import msvcrt
        if msvcrt.kbhit():
            return msvcrt.getch().decode('utf-8')
        return None
    except:
        import sys, select
        dr, dw, de = select.select([sys.stdin], [], [], 0)
        if dr:
            return sys.stdin.read(1)
        return None


# ROTATION PID ------------------------------------------------------------
KP = 0.4
KI = 0.0
KD = 20

K_ROT = .8
K_ROT_D = 7
K_ROT_I = 0.01

MIN_ROT_POWER = 0
MAX_ROT_POWER = 100

last_error = 0.0
error_integral = 0.0

def wrap180(e):
    while e > 180:
        e -= 360
    while e < -180:
        e += 360
    return e

def pulse_move_linear(s, heading, power, ms):
    """
    heading: 0 = forward, 180 = backward
    power: linear power
    ms: duration of pulse
    """
    cmd = f"MOVE {heading} {power} 0 0"
    send_cmd(s, cmd)
    time.sleep(ms / 1000.0)
    send_cmd(s, "STOP")
    
def compute_rot(current_heading, desired_heading):
    global last_error, error_integral

    rot_error = wrap180(desired_heading + current_heading - 90)
    print("rot_error_after180", rot_error)

    if abs(rot_error) < 2.0:
        last_error = rot_error
        return 0

    p = K_ROT * rot_error
    error_integral += rot_error
    i = K_ROT_I * error_integral
    d = K_ROT_D * (rot_error - last_error)

    raw = p + i + d
    last_error = rot_error

    return int(raw)


def send_cmd(s, cmd):
    s.sendall((cmd.strip() + "\n").encode())


# ===================== LINEAR DISTANCE PID ==========================
LIN_KP = 0.002
LIN_KI = 0.0003
LIN_KD = 2

dist_last_err = 0
dist_err_int = 0
# ===================================================================

CLOSE_DIST = 8.0
ARRIVE_DIST = 2.0
STABLE_FRAMES = 3
ANGLE_TOL = 5

stable_counter = 0
dist_filt = None
ALPHA = 0.45


def move_to_points(points, s, video, K, dist_cv):

    w = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))

    map1, map2 = cv2.initUndistortRectifyMap(
        K, dist_cv, None, K, (w, h), cv2.CV_32FC1
    )

    SCALE = 0.5

    rot_stable_count = 0
    ROT_STABLE_FRAMES = 20

    stall_power = 20
    last_dist_drive = None
    STALL_THRESHOLD = 0.3
    MAX_STALL_POWER = 50

    # -----------------------------
    # NEW: Backup and forward offset state
    # -----------------------------
    BACKUP_DIST = 4.5
    BACKUP_POWER = 30
    backing_up = False
    backup_start = None

    FORWARD_DIST = 4.5         # NEW
    FORWARD_POWER = 30     # NEW
    forward_after_align = False
    forward_start = None

    idx = 0
    pen_is_down = False

    global dist_last_err, dist_err_int
    global last_error, error_integral
    global stable_counter

    aligning = False

    print("Press 'q' at any time to STOP and quit.")

    while idx < len(points):

        c = key_pressed()
        if c and c.lower() == 'q':
            print("\n[QUIT] stopping robot...")
            send_cmd(s, "STOP")
            send_cmd(s, "UP")
            return

        if points[idx] is None:
            if pen_is_down:
                print("PEN UP")
                send_cmd(s, "UP")
                pen_is_down = False
            idx += 1
            aligning = False
            backing_up = False
            forward_after_align = False
            continue

        ret, frame = video.read()
        if not ret:
            continue

        frame = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)
        small = cv2.resize(frame, None, fx=SCALE, fy=SCALE,
                           interpolation=cv2.INTER_AREA)

        pose, _ = get_pose(small)
        if pose is None:
            continue

        x, y, theta = pose
        target = np.array(points[idx])
        current = np.array([x, y])
        diff = target - current
        dist = np.linalg.norm(diff)

        # ======================================================
        # STATE 1: BACKING UP (NEW)
        # ======================================================
        if backing_up:
            moved = np.linalg.norm(current - backup_start)
            print(f"[BACKUP] moved {moved:.2f} / {BACKUP_DIST:.2f}")

            if moved < BACKUP_DIST:
                cmd = f"MOVE 180 {BACKUP_POWER} 0 0"   # backward
                send_cmd(s, cmd)
                coord_log.append(-BACKUP_POWER)
                continue
            else:
                print("[BACKUP] complete → PEN UP + ALIGN NEXT")
                send_cmd(s, "STOP")

                if pen_is_down:
                    send_cmd(s, "UP")
                    pen_is_down = False

                backing_up = False
                idx += 1

                if idx >= len(points):
                    break

                if points[idx] is None:
                    aligning = False
                else:
                    aligning = True
                    rot_stable_count = 0

                continue

        # ======================================================
        # If last point and close, finish
        # ======================================================
        if idx == len(points) - 1 and dist < 2 and not aligning:
            send_cmd(s, "STOP")
            break

        # ======================================================
        # Waypoint stable detection
        # ======================================================
        if dist < ARRIVE_DIST:
            stable_counter += 1
        else:
            stable_counter = 0

        # ======================================================
        # START BACKUP (NEW - WITH PEN UP FIRST)
        # ======================================================
        if stable_counter >= STABLE_FRAMES and not backing_up:
            print("Waypoint reached → PEN UP → start backup")

            # NEW: Pen up BEFORE backing up
            if pen_is_down:
                print("PEN UP (before backup)")
                send_cmd(s, "UP")
                pen_is_down = False

            # reset controllers
            last_error = 0
            error_integral = 0
            dist_err_int = 0
            dist_last_err = 0

            # begin backup phase
            backup_start = current.copy()
            backing_up = True
            aligning = False
            stable_counter = 0
            continue

        # ======================================================
        # STATE 2: FORWARD OFFSET AFTER ALIGNMENT (NEW)
        # ======================================================
        if forward_after_align:
            moved = np.linalg.norm(current - forward_start)
            print(f"[FORWARD OFFSET] moved {moved:.2f} / {FORWARD_DIST:.2f}")

            if moved < FORWARD_DIST:
                cmd = f"MOVE 0 {FORWARD_POWER} 0 0"   # forward
                send_cmd(s, cmd)
                coord_log.append(FORWARD_POWER)
                continue
            else:
                print("[FORWARD OFFSET] complete → PEN DOWN + resume drawing")
                send_cmd(s, "STOP")

                if not pen_is_down:
                    send_cmd(s, "D")
                    pen_is_down = True

                forward_after_align = False
                continue

        # ======================================================
        # ALIGNMENT MODE
        # ======================================================
        desired_angle = np.degrees(np.arctan2(diff[1], diff[0]))
        angle_error = wrap180(90 - (desired_angle + theta))

        if aligning and abs(angle_error) > ANGLE_TOL:
            direction = -1 if angle_error > 0 else 1
            pulse_ms = int(4 + 2 * abs(angle_error))
            rot_power = 60
            cmd = f"MOVE 0 0 {direction * rot_power} 0"
            send_cmd(s, cmd)
            time.sleep(pulse_ms / 1000.0)
            send_cmd(s, "STOP")
            continue

        if aligning and abs(angle_error) <= ANGLE_TOL:
            rot_stable_count += 1
        else:
            rot_stable_count = 0

        # ======================================================
        # ALIGNMENT COMPLETE → START FORWARD OFFSET (NEW)
        # ======================================================
        if aligning and rot_stable_count >= ROT_STABLE_FRAMES:
            print("Alignment stable → forward offset phase BEGIN")
            aligning = False
            rot_stable_count = 0

            forward_after_align = True
            forward_start = current.copy()
            continue

        # ======================================================
        # DRIVE MODE
        # ======================================================
        if not aligning:

            if not pen_is_down:
                send_cmd(s, "D")
                pen_is_down = True

            if last_dist_drive is None:
                last_dist_drive = dist

            if dist > last_dist_drive - STALL_THRESHOLD:
                stall_power = min(int(stall_power * 1.2), MAX_STALL_POWER)
            else:
                stall_power = 20

            last_dist_drive = dist
            power = stall_power

            cmd = f"MOVE 0 {power} 0 0"
            send_cmd(s, cmd)
            coord_log.append(power)

    send_cmd(s, "UP")
    send_cmd(s, "STOP")
    with open("err_log.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["theta_or_power"])
        for th in coord_log:
            w.writerow([th])
