# main_nav.py
import csv
import cv2
import socket
from zeus_nav import move_to_points
import time
import numpy as np
ESP32_IP = "192.168.4.1"
ESP32_PORT = 8888

def load_csv(path):
    pts = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line == "":
                pts.append(None)  # <-- MARK BLANK LINE (pen-up)
                continue
            x, y = map(float, line.split(","))
            pts.append((x, y))
    return pts

if __name__ == "__main__":
    # ------------------------------------------------------------
    # Load path
    # ------------------------------------------------------------
    points = load_csv("/Users/divija/Divi Drive/workplace/Princeton/Sem 5/Carlab/carlab_2025/star.csv")
    print("here")
    # ------------------------------------------------------------
    # Setup ESP32 socket
    # ------------------------------------------------------------
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((ESP32_IP, ESP32_PORT))

    # ------------------------------------------------------------
    # Setup camera
    # ------------------------------------------------------------
    video = cv2.VideoCapture(0)
    time.sleep(1)   # warm-up
    data = np.load("camera_calib.npz")
    K = data["K"]; dist = data["dist"]
    # ------------------------------------------------------------
    # Setup video writer for annotated frames
    # ------------------------------------------------------------
    VIDEO_OUT = "nav_run_output.mp4"
    FPS = 20

    ret, frame = video.read()
    if not ret:
        print("❌ Could not read initial frame from camera.")
        exit()
    frame = cv2.undistort(frame, K, dist, None, K)
    H, W = frame.shape[:2]
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(VIDEO_OUT, fourcc, FPS, (W, H))

    print(f"🎥 Saving navigation run to: {VIDEO_OUT}")

    # ------------------------------------------------------------
    # Run navigation
    # move_to_points must return annotated frame each cycle
    # ------------------------------------------------------------
    try:
        #print("here")
        move_to_points(points, s, video, writer, K, dist)  
        #            ↑ pass writer as argument
        
    except KeyboardInterrupt:
        print("⛔ Interrupted by user.")

    # ------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------
    writer.release()
    video.release()
    s.close()
    cv2.destroyAllWindows()

    print("✅ Navigation complete, video saved.")
