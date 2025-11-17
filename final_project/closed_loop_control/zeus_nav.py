# zeus_nav.py — distance-based forward power ONLY
import numpy as np
import socket, time
from aruco_localization import get_pose


def send_cmd(s, cmd):
    s.sendall((cmd.strip() + "\n").encode())


def move_to_points(points, s, video):

    # ---------------- TUNABLE CONSTANTS ----------------
    MAX_POWER = 100     # highest forward PWM
    MIN_POWER = 0     # minimum pwm to move
    K_DIST = 1.75      # controls how fast power scales with distance
    STOP_DIST = 1.0    # inches tolerance

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

        # ------------ ARRIVAL CHECK ------------
        if dist < STOP_DIST:
            print("Reached point → STOP")
            send_cmd(s, "STOP")
            idx += 1
            continue

        # ------------ FORWARD POWER ------------
        raw_power = K_DIST * dist
        power = int(np.clip(raw_power, MIN_POWER, MAX_POWER))

        print(f"Power = {power}   (raw {raw_power:.2f})")

        # ------------ SEND COMMAND -------------
        # MOVE <angle> <power> <rot> <drift>
        # angle = 0 (forward), rot=0 (no turning)
        cmd = f"MOVE 0 {power} 0 0"
        send_cmd(s, cmd)
        print("CMD:", cmd)
