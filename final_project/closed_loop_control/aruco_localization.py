# aruco_localization.py
import cv2
import numpy as np

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
def get_pose(frame):
    """
    Detect ArUco markers in the frame, compute robot's position (x, y)
    in world coordinates (cm) and its orientation theta (deg).

    Returns:
        (x, y, theta), frame_with_overlay
        or (None, frame) if not detected.
    """
    corners, ids, _ = cv2.aruco.detectMarkers(frame, ARUCO_DICT, parameters=PARAMS)
    if ids is None:
        print("none found")
        return None, frame

    ids = ids.flatten()
    cv2.aruco.drawDetectedMarkers(frame, corners, ids)

    # Compute centers of all detected markers
    centers = {i: np.mean(c[0], axis=0) for i, c in zip(ids, corners)}

    # Compute homography if all 4 corners visible
    H = None
    if all(i in centers for i in CORNER_IDS):
        H = compute_homography(centers)

    # If no homography yet or robot not visible, just show camera
    if H is None or ROBOT_ID not in centers:
        return None, frame

    # -------------------------------------------------
    # Compute robot position
    # -------------------------------------------------
    pt_img = np.array([[centers[ROBOT_ID][0], centers[ROBOT_ID][1], 1.0]]).T
    world_pt = H @ pt_img
    world_pt /= world_pt[2, 0]
    x, y = world_pt[0, 0], world_pt[1, 0]

    # -------------------------------------------------
    # Compute orientation (theta)
    # -------------------------------------------------
    robot_corners = None
    for i, c in zip(ids, corners):
        if i == ROBOT_ID:
            robot_corners = c[0]
            break
    if robot_corners is None:
        return None, frame

    # -------------------------------------------------
    # Compute orientation (theta) from local +Y axis
    # -------------------------------------------------

    # 0 = top-left, 3 = bottom-left → Y axis on robot
    c0 = robot_corners[0]
    c3 = robot_corners[3]

    p0_world = (H @ np.array([[c0[0], c0[1], 1]]).T)
    p3_world = (H @ np.array([[c3[0], c3[1], 1]]).T)

    p0_world /= p0_world[2, 0]
    p3_world /= p3_world[2, 0]

    # robot local +Y vector in world coordinates
    vwx = p0_world[0, 0] - p3_world[0, 0]
    vwy = p0_world[1, 0] - p3_world[1, 0]

    # angle relative to global +Y (0,1)
    theta_world = np.degrees(np.arctan2(vwx, vwy))

    # -------------------------------------------------
    # Draw overlay
    # -------------------------------------------------
    cv2.circle(frame, tuple(np.int32(centers[ROBOT_ID])), 6, (0, 255, 0), -1)
    cv2.putText(frame, f"X={x:.1f}cm Y={y:.1f}cm Th={theta_world:.1f}°",
                (int(centers[ROBOT_ID][0] + 10), int(centers[ROBOT_ID][1] - 10)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    return (x, y, theta_world), frame

# -------------------------------------------------
# DEMO VIEWER
# -------------------------------------------------
if __name__ == "__main__":
    print("🎥 Opening camera... Press 'q' to quit.")
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("❌ Error: could not open camera.")
        exit()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("⚠️ Frame capture failed.")
            continue

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
