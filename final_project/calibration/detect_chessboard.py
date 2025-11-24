import cv2
import numpy as np
import glob
import os
import os


script_dir = os.path.dirname(os.path.abspath(__file__))
pattern = os.path.join(script_dir, "calib_*.png")

image_files = glob.glob(pattern)
# --------------------------------------
# Chessboard parameters
# --------------------------------------
CHESSBOARD_SIZE = (9, 6)  # inner corners
SQUARE_SIZE = 0.027686       # meters (or consistent unit)

# --------------------------------------
# Load images
# --------------------------------------

image_files.sort()

if len(image_files) == 0:
    print("No calibration images found!")
    exit(1)

print("Found", len(image_files), "calibration images.")

# --------------------------------------
# Prepare object points for one board
# --------------------------------------
objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

objpoints = []  # 3D
imgpoints = []  # 2D

# --------------------------------------
# Detect corners
# --------------------------------------
for fname in image_files:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(
        gray,
        CHESSBOARD_SIZE,
        cv2.CALIB_CB_ADAPTIVE_THRESH +
        cv2.CALIB_CB_FAST_CHECK +
        cv2.CALIB_CB_NORMALIZE_IMAGE
    )

    if ret:
        # refine
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)

        objpoints.append(objp)
        imgpoints.append(corners2)

        print(f"{fname}: ✓")

    else:
        print(f"{fname}: ✗ corners not found")

# --------------------------------------
# Calibration
# --------------------------------------
ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints,
    imgpoints,
    gray.shape[::-1],
    None,
    None
)

print("\n======================")
print("Calibration Complete")
print("======================")
print("RMS error:", ret)
print("Camera Matrix K:\n", K)
print("Distortion Coefficients:", dist.ravel())

# --------------------------------------
# Save calibration
# --------------------------------------
np.savez("camera_calib.npz", K=K, dist=dist)
print("\nSaved calibration to camera_calib.npz")
