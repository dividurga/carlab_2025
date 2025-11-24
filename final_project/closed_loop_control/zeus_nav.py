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

    MAX_POWER = 30
    MIN_POWER = 0
    MIN_DRIVE_POWER = 0
    K_DIST = 40
    STOP_DIST = 3
    STOP_DIST_FINAL = 4
    idx = 0

    print("Press 'q' at any time to STOP and quit.")

    while idx < (len(points)):
    #while True:
        # ---- QUIT CHECK ----
        c = key_pressed()
        if c and c.lower() == 'q':
            print("\n[QUIT] q pressed → stopping robot...")
            send_cmd(s, "STOP")
            return  # exit function entirely

        # # ---------------------------------------------------------------
        # # PEN UP / DOWN HANDLING (minimal addition)
        # # ---------------------------------------------------------------
        # if points[idx] is None:
        #     print("PEN UP")
        #     send_cmd(s, "UP")
        #     idx += 1
        #     continue

        # if idx == 0 or points[idx-1] is None:
        #     print("PEN DOWN")
        #     send_cmd(s, "D")

        # ---------------------------------------------------------------
        # Tracking loop
        print("hi")
        ret, frame = video.read()
        frame= cv2.undistort(frame, K, dist_cv, None, K)
        if frame is None:
            print("Frame")
        pose, annotated = get_pose(frame)
        if pose is None:
            print("none")
            continue
        print("here")
        writer.write(annotated)
        x, y, theta = pose
        target = np.array(points[idx])   # SAFE because we've checked None
        current = np.array([x, y])
        diff = target - current
        dist = np.linalg.norm(diff)

        print("\n==============================")
        print(f"Target {idx}: {target}")
        print(f"Current: {current}")
        print(f"Distance: {dist:.2f}")

        if(idx == (len(points)-1) and dist < STOP_DIST_FINAL):
               send_cmd(s, "STOP")
               break
        # ARRIVAL
        if dist < STOP_DIST:
            idx += 1
            print("Waypoint reached")
            continue
        
        # DESIRED ANGLE
        desired_angle = np.degrees(np.arctan2(diff[1], diff[0]))
        angle_error = 90 - (desired_angle + theta)
       

        
        

        rot = compute_rot(theta)

        # # POWER
        # raw_power = K_DIST * np.power(dist, 0.3)
        # power = int(np.clip(raw_power, MIN_POWER, MAX_POWER))

        # if power < MIN_DRIVE_POWER:
        #     power = MIN_DRIVE_POWER

        power = 60
        print(f"Power = {power}, angle_error: {angle_error}")
        print(f"rot = {rot}")

        #cmd = f"MOVE {int(angle_error)} {int(power)} 0 0"
        cmd = f"MOVE {int(angle_error)} 60 {int(min(80, (rot)))} 0"
        send_cmd(s, cmd)
        # idx +=1
        # time.sleep(10)
        # send_cmd(s, "STOP")

