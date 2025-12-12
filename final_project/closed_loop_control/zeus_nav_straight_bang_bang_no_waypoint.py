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


# ------------------------------------------------------------
# ROTATION PID
# ------------------------------------------------------------
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


# ------------------------------------------------------------
# >>> Pulse-based linear movement helper
# ------------------------------------------------------------
def pulse_move_linear(s, heading_deg, power, ms):
    cmd = f"MOVE {heading_deg} {power} 0 0"
    send_cmd(s, cmd)
    time.sleep(ms / 1000.0)
    send_cmd(s, "STOP")


# ------------------------------------------------------------
# Distance PID (still unused but harmless)
# ------------------------------------------------------------
LIN_KP = 0.002
LIN_KI = 0.0003
LIN_KD = 2

dist_last_err = 0
dist_err_int = 0

ARRIVE_DIST = 3
STABLE_FRAMES = 3
ANGLE_TOL = 5

stable_counter = 0


def move_to_points(points, s, video, K, dist_cv):

    w = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))

    map1, map2 = cv2.initUndistortRectifyMap(
        K, dist_cv, None, K, (w, h), cv2.CV_32FC1
    )

    SCALE = 0.5

    rot_stable_count = 0
    ROT_STABLE_FRAMES = 20

    # ------------------------------------------------------------
    # Backup + forward offset parameters
    # ------------------------------------------------------------
    BACKUP_DIST = 2
    BACKUP_POWER = 30
    BACKUP_PULSE_MS = 40

    FORWARD_DIST = 4
    FORWARD_POWER = 30
    FORWARD_PULSE_MS = 40

    backing_up = False
    backup_start = None

    forward_after_align = False
    forward_start = None

    # previously used for stall logic — completely removed
    # last_dist_drive = None  

    idx = 0
    pen_is_down = False

    global dist_last_err, dist_err_int
    global last_error, error_integral
    global stable_counter

    aligning = False

    print("Press 'q' at any time to STOP and quit.")


    # ------------------------------------------------------------
    # MAIN LOOP
    # ------------------------------------------------------------
    while idx < len(points):

        c = key_pressed()
        if c and c.lower() == 'q':
            print("\n[QUIT] stopping robot...")
            send_cmd(s, "STOP")
            send_cmd(s, "UP")
            return


        # Stroke break
        if points[idx] is None:
            if pen_is_down:
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
        diff   = target - current
        dist   = np.linalg.norm(diff)


        # End of final point
        if idx == len(points) - 1 and dist < 2 and not aligning:
            send_cmd(s, "STOP")
            break


        # Waypoint stable detection
        if dist < ARRIVE_DIST:
            stable_counter += 1
        else:
            stable_counter = 0


        # ------------------------------------------------------------
        # START BACKUP
        # ------------------------------------------------------------
        if stable_counter >= STABLE_FRAMES and not backing_up:
            print("Waypoint reached → PEN UP → start backup")

            if pen_is_down:
                send_cmd(s, "UP")
                pen_is_down = False

            last_error = 0
            error_integral = 0

            backing_up = True
            backup_start = current.copy()

            # compute backward direction FROM waypoint
            backward_dir = backup_start - target
            backward_dir /= np.linalg.norm(backward_dir)

            # compute the precise point we want to reach
            desired_backup_point = target + backward_dir * BACKUP_DIST

            aligning = False
            stable_counter = 0
            continue


        # ------------------------------------------------------------
        # BACKUP STATE (precise backward-dist enforcement)
        # ------------------------------------------------------------
        if backing_up:
            error = np.linalg.norm(current - desired_backup_point)
            print(f"[BACKUP] dist_to_goal = {error:.2f}")

            if error > .15 and error < 6:   # tolerance threshold, adjust if needed
                pulse_move_linear(
                    s,
                    heading_deg=180,   # backward
                    power=BACKUP_POWER,
                    ms=BACKUP_PULSE_MS
                )
                continue

            else:
                print("[BACKUP COMPLETE] → start alignment")
                send_cmd(s, "STOP")
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



        # ------------------------------------------------------------
        # FORWARD OFFSET STATE (pulse-based)
        # ------------------------------------------------------------
        if forward_after_align:
            moved = np.linalg.norm(current - forward_start)
            print(f"[FORWARD OFFSET] {moved:.2f}/{FORWARD_DIST}")

            if moved < FORWARD_DIST:
                pulse_move_linear(s, heading_deg=0,
                                  power=FORWARD_POWER,
                                  ms=FORWARD_PULSE_MS)
                continue
            else:
                print("[OFFSET DONE] → PEN DOWN + resume drawing")
                send_cmd(s, "STOP")

                if not pen_is_down:
                    send_cmd(s, "D")
                    pen_is_down = True

                forward_after_align = False
                continue


        # ------------------------------------------------------------
        # ALIGNMENT MODE (unchanged)
        # ------------------------------------------------------------
        desired_angle = np.degrees(np.arctan2(diff[1], diff[0]))
        angle_error   = wrap180(90 - (desired_angle + theta))

        if aligning and abs(angle_error) > ANGLE_TOL:
            direction = -1 if angle_error > 0 else 1
            pulse_ms  = int(4 + 2 * abs(angle_error))
            rot_power = 60

            cmd = f"MOVE 0 0 {direction * rot_power} 0"
            send_cmd(s, cmd)
            time.sleep(pulse_ms / 1000)
            send_cmd(s, "STOP")
            continue

        if aligning and abs(angle_error) <= ANGLE_TOL:
            rot_stable_count += 1
        else:
            rot_stable_count = 0


        if aligning and rot_stable_count >= ROT_STABLE_FRAMES:
            print("Alignment stable → forward offset")
            aligning = False
            forward_after_align = True
            forward_start = current.copy()
            continue


        # ------------------------------------------------------------
        # DRIVE MODE (pulse-based, NO STALL LOGIC ANYMORE)
        # ------------------------------------------------------------
        if not aligning:

            if not pen_is_down:
                send_cmd(s, "D")
                pen_is_down = True

            # constant forward pulses
            DRIVE_POWER = 40
            DRIVE_PULSE_MS = 40

            pulse_move_linear(
                s, heading_deg=0,
                power=DRIVE_POWER,
                ms=DRIVE_PULSE_MS
            )
            coord_log.append(DRIVE_POWER)


    # END OF LOOP CLEANUP
    send_cmd(s, "UP")
    send_cmd(s, "STOP")

    with open("err_log.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["power"])
        for th in coord_log:
            w.writerow([th])
