import numpy as np
import socket, time, sys
from aruco_localization_fast import get_pose
import cv2
import csv

coord_log = []

# OS-portable non-blocking keyboard check
def key_pressed():
    try:
        # Windows
        import msvcrt
        if msvcrt.kbhit():
            return msvcrt.getch().decode('utf-8')
        return None
    except:
        # Unix / macOS
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


def compute_rot(current_heading, desired_heading):
    global last_error, error_integral

    rot_error = wrap180(desired_heading + current_heading-90)
    print("rot_error_after180", rot_error)

    if abs(rot_error) < 2.0:
        last_error = rot_error
        return 0

    # ----- P TERM -----
    p = K_ROT * rot_error

    # ----- I TERM -----
    error_integral += rot_error
    i = K_ROT_I * error_integral

    # ----- D TERM -----
    d = K_ROT_D * (rot_error - last_error)

    raw = p + i + d

    # update last_error AFTER computing derivative
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
CLOSE_DIST = 8.0        # start creep mode here
ARRIVE_DIST = 4.0       # must get this close to declare arrival
STABLE_FRAMES = 3       # require staying close for N frames
ANGLE_TOL = 5         # how well we must align before driving

stable_counter = 0
dist_filt = None
ALPHA = 0.45            # smoothing for noisy distance


def move_to_points(points, s, video, K, dist_cv):

    # --------------------------
    # FAST UNDISTORT MAPS
    # --------------------------
    w = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))

    map1, map2 = cv2.initUndistortRectifyMap(
        K, dist_cv, None, K, (w, h), cv2.CV_32FC1
    )

    # --------------------------
    # DOWNSAMPLE SCALE
    # --------------------------
    SCALE = 0.5
    INV_SCALE = 1.0 / SCALE

    rot_stable_count = 0
    ROT_STABLE_FRAMES = 20

    MAX_POWER = 130
    MIN_POWER = 0
    STOP_DIST_FINAL = 2

    # ---- NEW STALL DETECTION VARS ----
    stall_power = 20
    last_dist_drive = None
    STALL_THRESHOLD = 0.3     # how much dist must shrink per frame
    MAX_STALL_POWER = 50

    idx = 0
    pen_is_down = False

    global dist_last_err, dist_err_int
    global last_error, error_integral
    global stable_counter

    aligning = False

    print("Press 'q' at any time to STOP and quit.")

    while idx < len(points):

        # ------------------------------------------------------
        # QUIT CHECK
        # ------------------------------------------------------
        c = key_pressed()
        if c and c.lower() == 'q':
            print("\n[QUIT] stopping robot...")
            send_cmd(s, "STOP")
            send_cmd(s, "UP")
            return

        # ------------------------------------------------------
        # HANDLE STROKE BREAK
        # ------------------------------------------------------
        if points[idx] is None:
            if pen_is_down:
                print("PEN UP")
                send_cmd(s, "UP")
                pen_is_down = False
            idx += 1
            aligning = False
            continue

        # ------------------------------------------------------
        # FRAME READ → FAST UNDISTORT
        # ------------------------------------------------------
        ret, frame = video.read()
        if not ret:
            continue

        frame = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)

        # ------------------------------------------------------
        # DOWNSAMPLE FOR FASTER ARUCO
        # ------------------------------------------------------
        small = cv2.resize(frame, None, fx=SCALE, fy=SCALE,
                           interpolation=cv2.INTER_AREA)

        # ------------------------------------------------------
        # GET POSE
        # ------------------------------------------------------
        pose, _ = get_pose(small)
        if pose is None:
            continue

        x, y, theta = pose

        # ------------------------------------------------------
        # TARGET + DISTANCE
        # ------------------------------------------------------
        target = np.array(points[idx])
        current = np.array([x, y])
        diff = target - current
        dist = np.linalg.norm(diff)

        # ------------------------------------------------------
        # FINAL ENDPOINT
        # ------------------------------------------------------
        if idx == len(points) - 1 and dist < STOP_DIST_FINAL:
            send_cmd(s, "STOP")
            break

        # ------------------------------------------------------
        # ARRIVAL HYSTERESIS
        # ------------------------------------------------------
        if dist < ARRIVE_DIST:
            stable_counter += 1
        else:
            stable_counter = 0

        if stable_counter >= STABLE_FRAMES:
            print("Waypoint reached (stable)")

            last_error = 0
            error_integral = 0
            dist_err_int = 0
            dist_last_err = 0

            if not pen_is_down:
                print("PEN DOWN")
                send_cmd(s, "D")
                pen_is_down = True

            idx += 1
            stable_counter = 0
            aligning = True
            send_cmd(s, "UP")
            continue

        # ------------------------------------------------------
        # ANGLE + ROTATION PID (ALIGN PHASE)
        # ------------------------------------------------------
        desired_angle = np.degrees(np.arctan2(diff[1], diff[0]))
        angle_error = 90 - (desired_angle + theta)
        angle_error = wrap180(angle_error)

        print("angle err:", angle_error,
              "desired:", desired_angle,
              "theta:", theta,
              "dist:", dist)

        # --- IMPULSE-BASED ROTATION CONTROL ---
        # --- IMPULSE-BASED ROTATION CONTROL ---
        rot_error = angle_error
        if aligning and abs(rot_error) > ANGLE_TOL:
            # direction: left (>0) or right (<0)
            direction = -1 if rot_error > 0 else 1

            # scale pulse length based on how far we are
            pulse_ms = int(6 + 2 * abs(rot_error))

            rot_power = 60  # constant power

            cmd = f"MOVE 0 0 {direction * rot_power} 0"
            send_cmd(s, cmd)
            time.sleep(pulse_ms / 1000.0)

            send_cmd(s, "STOP")
            continue
        # Check if angle is within tolerance → accumulate stability frames
        if aligning and abs(rot_error) <= ANGLE_TOL:
            rot_stable_count += 1
        else:
            rot_stable_count = 0

        # If stable long enough → alignment complete → resume drive
        if aligning and rot_stable_count >= ROT_STABLE_FRAMES:
            print("Alignment stable → switching to drive mode")
            aligning = False
            rot_stable_count = 0
        

        # ------------------------------------------------------
        # CONSTANT POWER + STALL BOOST  (DRIVE MODE)
        # ------------------------------------------------------
        if not aligning:
            # Initialize last_dist_drive
            if last_dist_drive is None:
                last_dist_drive = dist

            # detect stall
            if dist > last_dist_drive - STALL_THRESHOLD:
                stall_power = min(int(stall_power * 1.2), MAX_STALL_POWER)
                print("STALL → boosting power:", stall_power)
            else:
                stall_power = 20  # reset when moving normally

            last_dist_drive = dist

            power = stall_power

            cmd = f"MOVE 0 {power} 0 0"
            send_cmd(s, cmd)
            coord_log.append(power)

    # ----------------------------------------------------------
    # END
    # ----------------------------------------------------------
    send_cmd(s, "UP")

    with open("err_log.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["theta_or_power"])
        for th in coord_log:
            w.writerow([th])
