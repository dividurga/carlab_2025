# aruco_localization.py
import cv2
import numpy as np
VIDEO_OUTPUT = "aruco_output.mp4"
VIDEO_FPS = 20
# -------------------------------------------------
# CONFIGURATION
# -------------------------------------------------
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
PARAMS = cv2.aruco.DetectorParameters()

# IDs of 4 fixed corner markers and robot marker
CORNER_IDS = [0, 1, 2, 3]
ROBOT_ID = 4  # marker ID on the robot

# Real-world coordinates (cm) of corner markers
WORLD_POINTS = {
    0: (0, 0),
    1: (80, 0),    # bottom-right
    2: (80, 45),   # top-right
    3: (0, 45)     # top-left
}

# -------------------------------------------------
# HELPER FUNCTIONS
# -------------------------------------------------
def compute_homography(corner_positions):
    img_pts = np.array([corner_positions[i] for i in CORNER_IDS], dtype=np.float32)
    world_pts = np.array([WORLD_POINTS[i] for i in CORNER_IDS], dtype=np.float32)
    H, _ = cv2.findHomography(img_pts, world_pts)
    return H

# -------------------------------------------------
# CORE FUNCTION
# -------------------------------------------------
last_H = None     # ← GLOBAL OR OUTSIDE VARIABLE
last_centers = None

def get_pose(frame):
    global last_H, last_centers

    corners, ids, _ = cv2.aruco.detectMarkers(frame, ARUCO_DICT, parameters=PARAMS)

    if ids is None:
        ids = []
    else:
        ids = ids.flatten()

    cv2.aruco.drawDetectedMarkers(frame, corners, ids)

    # Build centers dictionary
    centers = {i: np.mean(c[0], axis=0) for i, c in zip(ids, corners)}

    # -------------------------------------------------
    # Update homography only when all corners visible
    # -------------------------------------------------
    if all(i in centers for i in CORNER_IDS):
        last_centers = centers
        H = compute_homography(centers)
        last_H = H

    else:
        # If we don't see corners but we have a past H → re-use it
        H = last_H

    # If still no homography, return None
    if H is None:
        return None, frame

    # -------------------------------------------------
    # Robot marker must be visible
    # -------------------------------------------------
    if ROBOT_ID not in centers:
        return None, frame

    # -------------------------------------------------
    # Compute robot position using cached or fresh H
    # -------------------------------------------------
    robot_center = centers[ROBOT_ID]
    pt_img = np.array([[robot_center[0], robot_center[1], 1.0]]).T

    world_pt = H @ pt_img
    world_pt /= world_pt[2, 0]

    x, y = world_pt[0, 0], world_pt[1, 0]

    # -------------------------------------------------
    # Compute robot orientation in world
    # -------------------------------------------------
    # use latest corners or last_centers if needed
    robot_corners = None
    for i, c in zip(ids, corners):
        if i == ROBOT_ID:
            robot_corners = c[0]
            break

    if robot_corners is None:
        return None, frame

    c0 = robot_corners[0]
    c3 = robot_corners[3]

    p0w = (H @ np.array([[c0[0], c0[1], 1]]).T); p0w /= p0w[2, 0]
    p3w = (H @ np.array([[c3[0], c3[1], 1]]).T); p3w /= p3w[2, 0]

    vwx = p0w[0, 0] - p3w[0, 0]
    vwy = p0w[1, 0] - p3w[1, 0]

    theta_world = np.degrees(np.arctan2(vwx, vwy))

    # -------------------------------------------------
    # Draw overlay
    # -------------------------------------------------
    cv2.circle(frame, tuple(np.int32(robot_center)), 6, (0,255,0), -1)
    cv2.putText(frame, f"X={x:.1f} Y={y:.1f} Th={theta_world:.1f}",
                (int(robot_center[0] + 10), int(robot_center[1] - 10)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

    return (x, y, theta_world), frame

# -------------------------------------------------
# DEMO VIEWER
# -------------------------------------------------
if __name__ == "__main__":
    print("🎥 Opening camera... Press 'q' to quit.")
    cap = cv2.VideoCapture(0)
    data = np.load("camera_calib.npz")
    K = data["K"]; dist = data["dist"]
    if not cap.isOpened():
        print("❌ Error: could not open camera.")
        exit()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("⚠️ Frame capture failed.")
            continue
        frame= cv2.undistort(frame, K, dist, None, K)
        pose, vis = get_pose(frame)
        if pose:
            x, y, th = pose
            print(f"Pose: X={x:.1f} cm, Y={y:.1f} cm, θ={th:.1f}°")

        cv2.imshow("Aruco Localization", vis)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("✅ Closed camera.")
