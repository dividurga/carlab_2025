# zeus_nav.py
import numpy as np
import socket, time
from aruco_localization import get_pose

def send_cmd(s, cmd):
    s.sendall((cmd.strip()+"\n").encode())

def move_to_points(points, s, video):
    idx = 0
    while idx < len(points):
        ret, frame = video.read()
        pose, _ = get_pose(frame)
        if pose is None:
            continue
        x, y, theta = pose
        # by convention, bot facing forward is 90 degrees in our vision code
        # but it is 0 in the robot frame

        target = np.array(points[idx])
        print("target: ", target)
        current = np.array([x, y])
        print("current: ", current)
        diff = target - current
        dist = np.linalg.norm(diff)
        if dist < 1:  # 1 inch tolerance
            idx += 1
            send_cmd(s, "STOP")
            continue
        print("delta x", diff[0])
        print("delta y", diff[1])
        desired_angle = np.degrees(np.arctan2(diff[1], diff[0]))
        print("desired angle: ", desired_angle)
        print("bot heading, ", theta)
        angle_error = 90 - (theta + desired_angle)
        print("angle error: ", angle_error)
        #angle_error = (angle_error + 180) % 360 - 180  # normalize
        #power = min(80, max(30, dist))  # simple scaling
        power = 80
        rot = int(np.clip(angle_error * 0.5, -50, 50))
        print(f"MOVE {int(angle_error)} {int(power)} {0} 0")
        send_cmd(s, f"MOVE {int(angle_error)} {60} {0} 0")
        #send_cmd(s, f"MOVE {int(desired_angle)} {int(power)} {rot} 0")
        #send_cmd(s, f"MOVE {int(0)} {int(0)} {90*idx} 0")
        

    