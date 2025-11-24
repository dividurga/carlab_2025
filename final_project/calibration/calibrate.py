import cv2, numpy as np

data = np.load("camera_calib.npz")
K = data["K"]; dist = data["dist"]

img = cv2.imread("final_project/calibration/calib_01.png")  # or any test image
h, w = img.shape[:2]
und = cv2.undistort(img, K, dist, None, K)

cv2.imshow("orig", img)
cv2.imshow("undistorted", und)
cv2.waitKey(0)

