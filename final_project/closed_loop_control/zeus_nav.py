import numpy as np
import socket, time
from aruco_localization import get_pose


# ROTATION PID
KP = .8
KI = 0.0
KD = 20

last_error = 0
error_integral = 0


def wrap180(e):
    while e > 180: e -= 360
    while e < -180: e += 360
    return e



def compute_rot(angle_error):
    global last_error, error_integral

    # Compute PID using the SAME angle-frame as movement
    rot_error = angle_error - 90   # <-- EXACTLY matches your world-coordinate math
    rot_error = wrap180(rot_error)

    d = rot_error - last_error
    error_integral += rot_error

    rot = KP * rot_error + KI * error_integral + KD * d
    rot = int(max(-100, min(100, rot)))

    last_error = rot_error
    return rot


def send_cmd(s, cmd):
    s.sendall((cmd.strip() + "\n").encode())


def move_to_points(points, s, video):

    MAX_POWER = 100
    MIN_POWER = 0
    K_DIST = 15
    STOP_DIST = 1

    idx = 0

    while idx < len(points):

        ret, frame = video.read()
        pose, _ = get_pose(frame)
        if pose is None:
            continue

        x, y, theta = pose
        target = np.array(points[idx])
        current = np.array([x, y])
        diff = target - current
        dist = np.linalg.norm(diff)

        print("\n==============================")
        print(f"Target {idx}: {target}")
        print(f"Current: {current}")
        print(f"Distance: {dist:.2f}")

        # ARRIVAL
        if dist < STOP_DIST:
            print("Reached point → STOP")
            send_cmd(s, "STOP")
            idx += 1
            continue

        # POWER
        raw_power = K_DIST * np.pow(dist, 0.3)
        power = int(np.clip(raw_power, MIN_POWER, MAX_POWER))
        print(f"Power = {power}   (raw {raw_power:.2f})")

        # DESIRED HEADING
        desired_angle = np.degrees(np.arctan2(diff[1], diff[0]))
        angle_error = 90 - (desired_angle + theta)

        # Compute rot from Arduino PID but using theta
        rot = compute_rot(angle_error)

        cmd = f"MOVE {int(angle_error)} {power} {0} 0"
        send_cmd(s, cmd)

        print("CMD:", cmd)
        print("angle_error =", angle_error, " rot =", 0)
