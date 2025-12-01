import numpy as np
import socket, time, sys
from aruco_localization import get_pose
import cv2
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

K_ROT = .6
K_ROT_D = .8
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
        p = 4.0 * K_ROT * rot_error
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

def move_to_points(points, s, video, writer, K, dist_cv):

    STOP_DIST = 3
    STOP_DIST_FINAL = 4

    idx = 0

    cached_target = None
    heading_aligned = False     # 👉 NEW
    strafe_angle = None         # 👉 NEW

    print("Press 'q' to quit")

    while idx < len(points):

        # Quit check
        c = key_pressed()
        if c and c.lower() == 'q':
            send_cmd(s, "STOP")
            return

        # Pose
        ret, frame = video.read()
        frame = cv2.undistort(frame, K, dist_cv, None, K)
        pose, annotated = get_pose(frame)
        if pose is None:
            continue

        writer.write(annotated)
        x, y, theta = pose
        current = np.array([x, y])

        
        if cached_target is None or not np.allclose(cached_target, points[idx]):
            cached_target = np.array(points[idx])
            heading_aligned = False    # force rotation again
            strafe_angle = None
            print(f"\n[Waypoint {idx}] target = {cached_target}")

        
        diff = cached_target - current
        dist = np.linalg.norm(diff)

        if idx == len(points)-1 and dist < STOP_DIST_FINAL:
            send_cmd(s, "STOP")
            break

        if dist < STOP_DIST:
            print("Waypoint reached → next")
            idx += 1
            cached_target = None
            continue

        
        if not heading_aligned:
            rot_error = wrap180(0 - theta)  # absolute 0° setpoint

            if abs(rot_error) > 3:   # still rotating
                rot_cmd = compute_rot(rot_error)
                cmd = f"MOVE 0 0 {int(min(80, rot_cmd))} 0"
                print(f"Rotating to North… θ={theta:.2f}, err={rot_error:.2f}")
                send_cmd(s, cmd)
                continue
            else:
                print("Heading aligned to 0° → moving to strafe phase")
                heading_aligned = True

        
        if strafe_angle is None:
            strafe_angle = np.degrees(np.arctan2(diff[1], diff[0]))
            print(f"Strafe angle for this waypoint = {strafe_angle:.2f}°")

        #
        power = 70
        cmd = f"MOVE {int(strafe_angle)} {power} 0 0"
        print(f"Strafing toward waypoint. dist={dist:.2f}")
        send_cmd(s, cmd)

