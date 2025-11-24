import cv2
import time
import os

# --------------------------------------
# SOUND FUNCTION (macOS)
# --------------------------------------
def play_beep():
    # built-in macOS sounds: Glass, Hero, Ping, Pop, Tink, etc.
    os.system('afplay /System/Library/Sounds/Glass.aiff')


# --------------------------------------
# CONFIGURATION
# --------------------------------------
CAMERA_INDEX = 0
NUM_IMAGES = 30
WAIT_TIME = 5

# --------------------------------------
# Create output directory (current folder)
# --------------------------------------
script_dir = os.path.dirname(os.path.abspath(__file__))
print("Saving images to:", script_dir)

# --------------------------------------
# Open camera
# --------------------------------------
cap = cv2.VideoCapture(CAMERA_INDEX)

if not cap.isOpened():
    print("ERROR: Cannot open camera", CAMERA_INDEX)
    exit(1)

print("Camera opened. Press Ctrl+C to stop early.")

# --------------------------------------
# Capture loop
# --------------------------------------
for i in range(1, NUM_IMAGES + 1):
    print(f"\nPosition your chessboard... capturing image {i}/{NUM_IMAGES} in {WAIT_TIME} seconds...")
    time.sleep(WAIT_TIME)

    ret, frame = cap.read()
    if not ret:
        print("ERROR: Failed to capture frame.")
        continue

    filename = os.path.join(script_dir, f"calib_{i:02d}.png")
    cv2.imwrite(filename, frame)

    print(f"Saved: {filename}")

    # Play sound to notify capture
    play_beep()

    # Show preview
    cv2.imshow("Captured Image", frame)
    cv2.waitKey(500)

print("\nDone! All images captured.")
cap.release()
cv2.destroyAllWindows()
