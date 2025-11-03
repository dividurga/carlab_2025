import cv2

# Choose the same dictionary you'll use in detection
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# IDs of markers to generate
marker_ids = [0, 1, 2, 3, 4]

# Marker size in pixels (for printing at high resolution)
marker_size = 400  # 600×600 px = ~5 cm marker if printed at 300 dpi

for marker_id in marker_ids:
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)
    filename = f"aruco_{marker_id}.png"
    cv2.imwrite(filename, marker_img)
    print(f"Saved {filename}")
