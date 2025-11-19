# zeus_client.py
import socket
import time

ESP32_IP = "192.168.4.1"  # ESP32 AP default IP if you used AP mode
ESP32_PORT = 8888

def send_cmd(s, cmd):
    s.sendall((cmd.strip()+"\n").encode('utf-8'))
    # try to read a response (non-blocking-ish)
    s.settimeout(0.5)
    try:
        r = s.recv(1024)
        if r:
            print("reply:", r.decode().strip())
    except socket.timeout:
        pass

if __name__ == "__main__":
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((ESP32_IP, ESP32_PORT))
    # send_cmd(s, "CALIBRATE")      # calibrate
    # print("calibrated")
    print("rst")
    send_cmd(s, "RST")
    
