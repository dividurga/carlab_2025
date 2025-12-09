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
LIN_KP = 0.002
LIN_KI = 0.0003
LIN_KD = 2

dist_last_err = 0
dist_err_int = 0
# ===================================================================
CLOSE_DIST = 8.0        # start creep mode here
ARRIVE_DIST = 3.0       # must get this close to declare arrival
STABLE_FRAMES = 3       # require staying close for N frames

stable_counter = 0
dist_filt = None
ALPHA = 0.45            # smoothing for noisy distance

def move_to_points(points, s, video, writer, K, dist_cv):

    MAX_POWER = 100         # allow PID to add on top
    MIN_POWER = 0          # always give some thrust
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

        #------------------- PEN LOGIC --------------------
        # Stroke break → None means end of stroke
        if points[idx] is None:
            if pen_is_down:
                print("PEN UP")
                send_cmd(s, "UP")
                pen_is_down = False
            idx += 1
            continue

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
        print("now here")
        if idx == len(points)-1 and dist < STOP_DIST_FINAL:
            send_cmd(s, "STOP")
            break

        # # ------------------ FILTER DISTANCE (reduces jitter → fewer oscillations) ------------------
        # if dist_filt is None:
        #     dist_filt = dist
        # else:
        #     dist_filt = ALPHA * dist + (1 - ALPHA) * dist_filt

        # dist = dist_filt
        # -------------------------------------------------------------------------------------------

        # ---------- HYSTERESIS: Only declare arrival if stable for N consecutive frames ----------
        if dist < ARRIVE_DIST:
            stable_counter += 1
        else:
            stable_counter = 0

        if stable_counter >= STABLE_FRAMES:
            print("Waypoint reached (stable)")

            #send_cmd(s, "STOP")
            #time.sleep(0.15)

            # Reset ROTATION PID memory  ← FIX #1
            global last_error, error_integral
            last_error = 0
            error_integral = 0

            # reset distance PID too
            dist_err_int = 0
            dist_last_err = 0

            if not pen_is_down:
                print("PEN DOWN")
                send_cmd(s, "D")
                pen_is_down = True

            idx += 1
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

                # ===============================================================
        # (2) BASE POWER FROM MOVEMENT TYPE DUE TO ASYMMETRY IN MECHANICAL RESPONSE
        # ===============================================================


        centre = (angle_error >= -40 and angle_error <= 40) or (angle_error >= 140 and angle_error <= 220)
        
        base_power = 60 if centre else 120
        # ===============================================================

        # ===============================================================
        #  (1) LINEAR PID CONTROL BASED ON DISTANCE
        #      Extra damping + softer push for forward/back
        # ===============================================================

        # P
        p_lin = LIN_KP * dist_err

        # I
        dist_err_int += dist_err
        i_lin = LIN_KI * dist_err_int

        # D  ---- more damping when NOT sideways ----
        kd_eff = LIN_KD * (25.0 if centre else 1.0)
        d_lin = kd_eff * (dist_err - dist_last_err)
        dist_last_err = dist_err

        # soften P when going mostly straight (forward/back)
        # -------------------- CREEP MODE NEAR TARGET --------------------
        if centre:
            if dist < CLOSE_DIST:
                base_power = 40       # very gentle forward creep
                power_cap = 50
                # kill integral + soften derivative near goal
                i_lin = 0
                kd_eff = LIN_KD * 3.0
            else:
                base_power = 60
                power_cap = 60
        else:
            base_power = 100
            power_cap = MAX_POWER
        # ----------------------------------------------------------------


        pid_out = p_lin + i_lin + d_lin
        # ===============================================================

        # # Final commanded power
        # power_cap = 60 if centre else MAX_POWER   # forward/back is capped
        # power = int(np.clip(base_power + pid_out, MIN_POWER, power_cap))

        # ===============================================================
        # Smooth baseline power (does not change PID behavior)
        # theta_rad = np.deg2rad(angle_error)

        # # smooth gain: 0 at 0°, 1 at ±90°, 0 at ±180°
        # gain = abs(np.sin(theta_rad))

        # base_power = 35 + 55 * gain  # between 40 and 100
        # Final commanded power
        #power = int(np.clip(base_power + pid_out, MIN_POWER, MAX_POWER))
        power = int(np.clip(base_power + pid_out, MIN_POWER, power_cap))

        cmd = f"MOVE {int(angle_error)} {power} {int(min(100, rot))} 0"
        send_cmd(s, cmd)
        coord_log.append(power)
        #print(cmd)
        print("angle err, ", angle_error, "desired angle", desired_angle, "theta", theta, "power", power)
    send_cmd(s, "UP")

    with open("err_log.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["theta"])
        for th in coord_log:
            w.writerow([th])
    