# main_nav.py
import csv
import cv2
import socket
from zeus_nav import move_to_points
import time
ESP32_IP = "192.168.4.1"
ESP32_PORT = 8888

def load_csv(path):
    with open(path) as f:
        rdr = csv.reader(f)
        return [(float(x), float(y)) for x, y in rdr]

if __name__ == "__main__":
    points = load_csv("/Users/divija/Divi Drive/workplace/Princeton/Sem 5/Carlab/carlab_2025/star.csv")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((ESP32_IP, ESP32_PORT))
    video = cv2.VideoCapture(1)
    time.sleep(1)
    move_to_points(points, s, video)
