import numpy as np
import socket, time, sys
from aruco_localization import get_pose
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

K_ROT = 1.6
K_ROT_D = 1.6
K_ROT_I = 0 # this is fucking stuff up
MIN_ROT_POWER = 0
MAX_ROT_POWER = 60

last_error = 0.0
error_integral = 0.0


def wrap180(e):
    while e > 180: e -= 360
    while e < -180: e += 360
    return e


def compute_rot(angle_error):
    global last_error, error_integral

    rot_error = wrap180(angle_error)
    print("rot_error_after180", rot_error)

    # # # small deadband
    # if abs(rot_error) < 15.0:
    #     last_error = rot_error
    #     return rot_error*4
    if abs(rot_error) < 2.0:
        last_error = rot_error
        return 0
    
    if abs(rot_error) < 15:
        p = K_ROT * rot_error
    else:
        p = K_ROT * rot_error
    # ----- P TERM -----

    # ----- I TERM -----
    error_integral += rot_error
    i = K_ROT_I * error_integral

    # ----- D TERM -----
    d = K_ROT_D * (rot_error - last_error)

    raw = p + i + d

    # update last_error AFTER computing derivative
    last_error = rot_error
    # if abs(rot_error) > 30.0:
    #     last_error = rot_error
    #     return raw/4.0
    return int(raw)

def send_cmd(s, cmd):
    s.sendall((cmd.strip() + "\n").encode())


# ========================================================================

# ===================== LINEAR DISTANCE PID ==========================
LIN_KP = 0.8
LIN_KI = 0.02
LIN_KD = 0.3

dist_last_err = 0
dist_err_int = 0
# ===================================================================


def move_to_points(points, s, video, writer, K, dist_cv):

    MAX_POWER = 150         # allow PID to add on top
    MIN_POWER = 40          # always give some thrust
    STOP_DIST = 3
    STOP_DIST_FINAL = 3
    idx = 0
    pen_is_down = False
 
    global dist_last_err, dist_err_int
    print("Press 'q' at any time to STOP and quit.")

    while idx < (len(points)):

        # ---- QUIT CHECK ----
        c = key_pressed()
        if c and c.lower() == 'q':
            print("\n[QUIT] q pressed → stopping robot...")
            send_cmd(s, "STOP")
            send_cmd(s, "UP")
            return

        #------------------- PEN UP / DOWN HANDLING --------------------
        if points[idx] is None:
            if pen_is_down:
                print("PEN UP")
                send_cmd(s, "UP")
                pen_is_down = False
            idx += 1
            continue

        if not pen_is_down:
            print("PEN DOWN")
            send_cmd(s, "D")
            pen_is_down = True

        # ---------------------------------------------------------------
        # Tracking loop
        ret, frame = video.read()
        frame = cv2.undistort(frame, K, dist_cv, None, K)
        pose, annotated = get_pose(frame)
        if pose is None:
            continue

        writer.write(annotated)
        x, y, theta = pose
        target = np.array(points[idx])
        current = np.array([x, y])

        diff = target - current
        dist = np.linalg.norm(diff)

        if idx == len(points)-1 and dist < STOP_DIST_FINAL:
            send_cmd(s, "STOP")
            break

        # ARRIVAL
        if dist < STOP_DIST:
            idx += 1
            print("Waypoint reached")
            dist_err_int = 0  # reset integral
            continue

        # DESIRED ANGLE
        desired_angle = np.degrees(np.arctan2(diff[1], diff[0]))
        angle_error = 90 - (desired_angle + theta)

        # ROTATION PID
        rot = compute_rot(theta)

        # ===============================================================
        #  (1) LINEAR PID CONTROL BASED ON DISTANCE
        # ===============================================================
        dist_err = dist

        # P
        p_lin = LIN_KP * dist_err

        # I
        dist_err_int += dist_err
        i_lin = LIN_KI * dist_err_int

        # D
        d_lin = LIN_KD * (dist_err - dist_last_err)
        dist_last_err = dist_err

        pid_out = p_lin + i_lin + d_lin
        # ===============================================================

        # ===============================================================
        # (2) BASE POWER FROM MOVEMENT TYPE DUE TO ASYMMETRY IN MECHANICAL RESPONSE
        # ===============================================================
        sideways = ((angle_error <= 120 and angle_error >= 60) or 
                    (angle_error >= -120 and angle_error <= -60))

        base_power = 100 if sideways else 60
        # ===============================================================

        # Final commanded power
        power = int(np.clip(base_power + pid_out, MIN_POWER, MAX_POWER))

        cmd = f"MOVE {int(angle_error)} {power} {int(min(80, rot))} 0"
        send_cmd(s, cmd)
        print(cmd)

    send_cmd(s, "UP")

    with open("err_log.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["theta"])
        for th in coord_log:
            w.writerow([th])
    
