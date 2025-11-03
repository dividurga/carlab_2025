import cv2
import numpy as np

# -------------------------------------------------
# CONFIGURATION
# -------------------------------------------------
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
PARAMS = cv2.aruco.DetectorParameters()

# IDs of the 4 corner markers and robot marker
CORNER_IDS = [0, 1, 2, 3]
ROBOT_ID = 4  # marker ID on the robot

# Real-world coordinates of the 4 corners (in cm)
WORLD_POINTS = {
    0: (0, 0),
    1: (30, 0),   # X = 30 cm
    2: (0, 20),   # Y = 20 cm
    3: (30, 20)
}

# -------------------------------------------------
# HELPER FUNCTION
# -------------------------------------------------
def compute_homography(corner_positions):
    img_pts = np.array([corner_positions[i] for i in CORNER_IDS], dtype=np.float32)
    world_pts = np.array([WORLD_POINTS[i] for i in CORNER_IDS], dtype=np.float32)
    H, _ = cv2.findHomography(img_pts, world_pts)
    return H

# -------------------------------------------------
# INPUT / OUTPUT SETUP
# -------------------------------------------------
video_path = "/Users/divija/Divi Drive/workplace/Princeton/Sem 5/Carlab/carlab_2025/final_project/test_video.mov"
cap = cv2.VideoCapture(video_path)

if not cap.isOpened():
    print("Error: Could not open video file.")
    exit()

fps = cap.get(cv2.CAP_PROP_FPS)
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# Set up output video writer (same size & fps as input)
out_path = "tracked_output.mp4"
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # codec for .mp4
out = cv2.VideoWriter(out_path, fourcc, fps, (width, height))

print(f"Saving processed video to {out_path}")

# -------------------------------------------------
# MAIN LOOP
# -------------------------------------------------
while True:
    ret, frame = cap.read()
    if not ret:
        break

    corners, ids, _ = cv2.aruco.detectMarkers(frame, ARUCO_DICT, parameters=PARAMS)
    if ids is not None:
        ids = ids.flatten()
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # Get marker centers
        centers = {}
        for marker_id, corner in zip(ids, corners):
            center = np.mean(corner[0], axis=0)
            centers[marker_id] = center

        # Compute homography if all corners found
        if all(i in centers for i in CORNER_IDS):
            H = compute_homography(centers)

            if ROBOT_ID in centers:
                robot_center = np.array([[centers[ROBOT_ID][0]], [centers[ROBOT_ID][1]], [1]])
                world_coord = H @ robot_center
                world_coord /= world_coord[2]

                x, y = world_coord[0][0], world_coord[1][0]
                cv2.circle(frame, tuple(np.int32(centers[ROBOT_ID])), 6, (0, 255, 0), -1)
                cv2.putText(frame, f"x={x:.1f}cm, y={y:.1f}cm",
                            (int(centers[ROBOT_ID][0] + 10), int(centers[ROBOT_ID][1] - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # Show and save frame
    cv2.imshow("Aruco Tracking", frame)
    out.write(frame)

    if cv2.waitKey(int(1000 / fps)) & 0xFF == ord('q'):
        break

# -------------------------------------------------
# CLEANUP
# -------------------------------------------------
cap.release()
out.release()
cv2.destroyAllWindows()

print("✅ Done! Video saved as:", out_path)
