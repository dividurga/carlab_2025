# main_nav.py
import csv
import cv2
import socket
from zeus_nav import move_to_points
import time
ESP32_IP = "192.168.4.1"
ESP32_PORT = 8888

def load_csv(path):
    pts = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line == "":
                pts.append(None)  # <-- MARK BLANK LINE
                continue
            x, y = map(float, line.split(","))
            pts.append((x, y))
    return pts

if __name__ == "__main__":
    points = load_csv("/Users/divija/Divi Drive/workplace/Princeton/Sem 5/Carlab/carlab_2025/star.csv")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((ESP32_IP, ESP32_PORT))
    video = cv2.VideoCapture(0)
    time.sleep(1)
    move_to_points(points, s, video)
