# very very very good code oolala so sexy
import cv2
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import scipy.ndimage
import os

# -------------------------------------------------
# Initialize MediaPipe
# -------------------------------------------------
mp_face_mesh = mp.solutions.face_mesh
mp_selfie_segmentation = mp.solutions.selfie_segmentation

HAIR_MODEL_PATH = '\Users\cynt1\Downloads\college\fall25\ece302\carlab_2025\final_project\hair_segmenter.tflite'

# -------------------------------------------------
# Facial feature landmark groups
# -------------------------------------------------
FACIAL_FEATURES = {
    # Eyes (outline of left and right eyes)
    "left_eye": [33, 7, 163, 144, 145, 153, 154, 155, 133,
                 173, 157, 158, 470, 160, 161, 246],
    "right_eye": [263, 249, 390, 373, 374, 380, 381, 382,
                  362, 398, 384, 385, 475, 387, 388, 466],

    # Eyebrows
    "left_eyebrow": [70, 63, 105, 66, 107, 55, 65, 52, 53, 70],
    "right_eyebrow": [336, 296, 334, 293, 300, 276, 283, 282, 295, 285, 336],

    # Upper / lower eyelid contours for clipped eyeballs
    "top_eye": [414, 286, 258, 257, 259, 260, 467],
    "bottom_eye": [453, 452, 451, 450, 449, 448],
    "top_eye_left": [190, 56, 28, 27, 29, 30, 247],
    "bottom_eye_left": [228, 229, 230, 231, 232, 233],

    # Lips
    "outer_lips": [61, 146, 91, 181, 84, 17, 314, 405,
                   321, 375, 291, 308, 324, 318, 402,
                   317, 14, 87, 178, 88, 95, 78, 191, 80, 81, 82, 13, 312, 311, 310, 415, 308],
    "inner_lips": [78, 95, 88, 178, 87, 14, 317, 402,
                   318, 324, 308, 291, 375, 321, 405, 314, 17, 84],
    "upper_lip": [61, 40, 37, 0, 267, 270, 409, 291],
    "lower_lip": [61, 146, 91, 181, 84, 17, 314, 405, 321, 375, 291],

    # Chin / jawline
    "chin": [58, 138, 172, 136, 150, 149, 176, 148, 152, 377, 400, 378, 379],
    # actual nose:
    "nose_left": [122, 196, 236, 198, 209, 49, 48, 64, 98],
    "nose_right": [420, 360, 278, 294, 327, 326],
    "nostrils_left" : [98, 240, 75, 60, 99, 97],
    "nostrils_right" : [370, 354, 458, 290],
    "nose_center": [238, 241,242,370, 354, 94],
    # Nose to lips (philtrum)
    "nose_to_lips_left": [203, 206, 216, 57],
    "nose_to_lips_right": [423, 426, 436, 287],

    # Mid-eye centers for eyeball placement
    "left_eye_center": [468],
    "right_eye_center": [473],
}   


EYEBALLS = {
    "left_iris": [468, 469, 470, 471],
    "right_iris": [473, 474, 475, 476],
}

# -------------------------------------------------
# Helpers
# -------------------------------------------------
def draw_feature(image, landmarks, indices, color=0, thickness=1):
    pts = [(int(landmarks[i].x * image.shape[1]),
            int(landmarks[i].y * image.shape[0])) for i in indices]
    for i in range(len(pts) - 1):
        cv2.line(image, pts[i], pts[i + 1], color, thickness)

def draw_eyeball_clipped(image, landmarks, iris_indices, top_eyelid_index, bottom_eyelid_index, color=0):
    H, W = image.shape[:2]
    pts = np.array([[landmarks[i].x * W, landmarks[i].y * H] for i in iris_indices])
    center = np.mean(pts, axis=0)
    radius = np.mean(np.linalg.norm(pts - center, axis=1))
    upper_y = landmarks[top_eyelid_index].y * H
    bottom_y = landmarks[bottom_eyelid_index].y * H
    for theta in np.linspace(0, 2*np.pi, 200):
        x = int(center[0] + radius * np.cos(theta))
        y = int(center[1] + radius * np.sin(theta))
        if upper_y <= y <= bottom_y and 0 <= y < H and 0 <= x < W:
            image[y, x] = color

# -------------------------------------------------
# Main
# -------------------------------------------------
def image_to_minimal_line_drawing_simplified(image_path, output_path):
    print(f"Processing: {image_path}")
    img = cv2.imread(image_path)
    if img is None:
        print("Error: Image not found.")
        return
    H, W, _ = img.shape
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    final_line_drawing = np.ones((H, W), dtype=np.uint8) * 255

    # ---- FaceMesh ----
    with mp_face_mesh.FaceMesh(
        static_image_mode=True, max_num_faces=1,
        refine_landmarks=True, min_detection_confidence=0.5
    ) as face_mesh:
        res = face_mesh.process(img_rgb)
        if not res.multi_face_landmarks:
            print("No face found."); return
        face_landmarks = res.multi_face_landmarks[0]

    # ---- Facial features ----
    for name, idxs in FACIAL_FEATURES.items():
        if "center" in name: continue
        draw_feature(final_line_drawing, face_landmarks.landmark, idxs)
    for iris_name, iris_indices in EYEBALLS.items():
        if "left" in iris_name:
            draw_eyeball_clipped(final_line_drawing, face_landmarks.landmark, iris_indices, 160, 145)
        else:
            draw_eyeball_clipped(final_line_drawing, face_landmarks.landmark, iris_indices, 385, 374)

    # ---- Hair segmentation ----
    hair_outline = np.ones((H, W), dtype=np.uint8) * 255
    hair_mask = None
    try:
        base_opts = python.BaseOptions(model_asset_path=HAIR_MODEL_PATH)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=img_rgb)
        opts = vision.ImageSegmenterOptions(
            base_options=base_opts,
            running_mode=vision.RunningMode.IMAGE,
            output_category_mask=True
        )
        with vision.ImageSegmenter.create_from_options(opts) as segmenter:
            seg = segmenter.segment(mp_image)
            cat_mask = seg.category_mask.numpy_view()
            hair_mask = np.where(cat_mask == 1, 255, 0).astype(np.uint8)
            hair_mask = cv2.resize(hair_mask, (W, H), interpolation=cv2.INTER_NEAREST)
            cs, _ = cv2.findContours(hair_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            for c in cs:
                eps = 0.002 * cv2.arcLength(c, True)
                approx = cv2.approxPolyDP(c, eps, True)
                cv2.drawContours(hair_outline, [approx], -1, 0, 1)
            hair_outline = cv2.ximgproc.thinning(255 - hair_outline)
            hair_outline = 255 - hair_outline
            final_line_drawing = np.minimum(final_line_drawing, hair_outline)
    except Exception as e:
        print(f"Hair model error: {e}")

    # ---- Selfie segmentation ----
    outline = np.ones((H, W), dtype=np.uint8) * 255
    with mp_selfie_segmentation.SelfieSegmentation(model_selection=1) as seg:
        res = seg.process(img_rgb)
        if res.segmentation_mask is not None:
            body_mask = (res.segmentation_mask > 0.5).astype(np.uint8) * 255
            cs, _ = cv2.findContours(body_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            for c in cs:
                eps = 0.0015 * cv2.arcLength(c, True)
                approx = cv2.approxPolyDP(c, eps, True)
                cv2.drawContours(outline, [approx], -1, 0, 1)

    # ---- Hair-priority merge ----
    if hair_mask is not None:
        hair_bin = cv2.threshold(hair_outline, 127, 255, cv2.THRESH_BINARY_INV)[1]
        body_bin = cv2.threshold(outline, 127, 255, cv2.THRESH_BINARY_INV)[1]
        dist_to_hair = cv2.distanceTransform(255 - hair_bin, cv2.DIST_L2, 5)
        proximity_thresh = 20  # px; tune this
        body_bin[dist_to_hair < proximity_thresh] = 0
        outline = 255 - body_bin

    # ---- Combine & finalize ----
    final_line_drawing = np.minimum(final_line_drawing, outline)
    binary = cv2.threshold(final_line_drawing, 127, 255, cv2.THRESH_BINARY_INV)[1]
    skeleton = cv2.ximgproc.thinning(binary)
    final_line_drawing = 255 - skeleton

    cv2.imwrite(output_path, final_line_drawing)
    print(f"✅ Saved single-pixel line drawing to: {output_path}")



# -------------------------------------------------
# Camera + face capture parameters
# -------------------------------------------------
CAPTURE_PATH = "captured_face.png"
CENTER_TOLERANCE = 0.1  # 10% of frame width/height
FACE_DETECT_CONF = 0.6

mp_face = mp.solutions.face_detection

def capture_centered_face(output_path=CAPTURE_PATH):
    cap = cv2.VideoCapture(0)
    detector = mp_face.FaceDetection(model_selection=0, min_detection_confidence=FACE_DETECT_CONF)
    print("📷 Starting camera. Center your face to capture automatically...")

    countdown_start = None
    countdown_time = 3  # seconds
    captured = False

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Camera read failed.")
            break

        # 🪞 Mirror image for a natural view
        frame = cv2.flip(frame, 1)

        H, W, _ = frame.shape
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = detector.process(rgb)

        centered = False
        if results.detections:
            for det in results.detections:
                bbox = det.location_data.relative_bounding_box
                cx = bbox.xmin + bbox.width / 2
                cy = bbox.ymin + bbox.height / 2
                dx = abs(cx - 0.5)
                dy = abs(cy - 0.5)
                centered = dx < CENTER_TOLERANCE and dy < CENTER_TOLERANCE

                # Draw guide box
                cv2.rectangle(frame,
                              (int((0.5 - CENTER_TOLERANCE)*W), int((0.5 - CENTER_TOLERANCE)*H)),
                              (int((0.5 + CENTER_TOLERANCE)*W), int((0.5 + CENTER_TOLERANCE)*H)),
                              (0,255,0) if centered else (0,0,255), 2)
                cv2.putText(frame, "Align your face in green box", (30,40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0) if centered else (0,0,255), 2)

        # Countdown logic
        if centered:
            if countdown_start is None:
                countdown_start = cv2.getTickCount() / cv2.getTickFrequency()
            elapsed = (cv2.getTickCount() / cv2.getTickFrequency()) - countdown_start
            remaining = int(countdown_time - elapsed)
            if remaining > 0:
                cv2.putText(frame, f"Hold still... {remaining}", (int(W/2 - 100), int(H/2)),
                            cv2.FONT_HERSHEY_DUPLEX, 2, (0,255,0), 4)
            else:
                cv2.putText(frame, "📸 Capturing!", (int(W/2 - 120), int(H/2)),
                            cv2.FONT_HERSHEY_DUPLEX, 2, (0,255,0), 4)

                # --- Flash effect ---
                cv2.imshow("Align Your Face", np.ones_like(frame)*255)
                cv2.waitKey(100)

                # --- Capture clean frame (mirrored) ---
                ret, clean_frame = cap.read()
                if ret:
                    clean_frame = cv2.flip(clean_frame, 1)  # mirror the saved version too
                    cv2.imwrite(output_path, clean_frame)
                else:
                    cv2.imwrite(output_path, frame)  # fallback

                captured = True
        else:
            countdown_start = None  # reset countdown if moved away

        cv2.imshow("Align Your Face", frame)
        if cv2.waitKey(1) & 0xFF == ord('q') or captured:
            break

    cap.release()
    cv2.destroyAllWindows()
    detector.close()

    if captured:
        print(f"✅ Captured clean mirrored image saved to {output_path}")
        return output_path
    else:
        print("❌ No capture made.")
        return None

# -------------------------------------------------
# Main combined flow
# -------------------------------------------------
def capture_and_generate():
    captured_path = capture_centered_face()
    if captured_path and os.path.exists(captured_path):
        # Run the full pipeline on captured image
        line_path = "captured_line.png"
        csv_path = "captured_coords.csv"
        video_path = "captured_draw.mp4"
        image_to_minimal_line_drawing_simplified(captured_path, line_path)

# -------------------------------------------------
# Run
# -------------------------------------------------
if __name__ == "__main__":
    capture_and_generate()
