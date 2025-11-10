import cv2
import numpy as np
import csv
import os
from scipy.spatial import distance_matrix
from python_tsp.heuristics import solve_tsp_local_search
import pandas as pd

# -------------------------------------------------
# Core: TSP on skeleton pixels
# -------------------------------------------------
def skeleton_to_tsp_path(
    skeleton_img: np.ndarray,
    max_points: int = 1500,
    jump_thresh: float = 10.0,
):
    """
    Given a skeletonized image (0=black, 255=white),
    return a list of (x, y, pen) following a TSP tour.

    - max_points: cap on sampled pixels used as TSP nodes
    - jump_thresh: distance threshold above which we mark pen-up
    """
    # 1) Get all skeleton pixels
    binary = (skeleton_img < 128).astype(np.uint8)
    ys, xs = np.where(binary == 1)

    if len(xs) == 0:
        raise ValueError("No skeleton pixels found in image.")

    points = np.stack([xs, ys], axis=1)  # shape (N, 2)

    # 2) Subsample if too many points
    N = len(points)
    if N > max_points:
        idx = np.random.choice(N, max_points, replace=False)
        points = points[idx]
        print(f"Subsampled from {N} to {len(points)} points for TSP.")

    # 3) Build distance matrix
    dist_matrix = distance_matrix(points, points)

    # 4) Solve TSP using python-tsp heuristic
    print("Running TSP solver...")
    permutation, total_dist = solve_tsp_local_search(dist_matrix)
    print(f"TSP tour length: {total_dist:.2f} over {len(points)} points.")

    ordered_points = points[permutation]

    # 5) Convert tour to (x, y, pen)
    coords = []
    # First point: pen up -> move there
    x0, y0 = ordered_points[0]
    coords.append((int(x0), int(y0), 1))

    for i in range(1, len(ordered_points)):
        x, y = ordered_points[i]
        prev = ordered_points[i - 1]
        step_dist = np.linalg.norm(ordered_points[i] - prev)

        # If it's a small local move, keep pen down
        # If it's a big jump across the figure, lift pen
        pen = 2 if step_dist <= jump_thresh else 1
        coords.append((int(x), int(y), pen))

    return coords


def save_coords_to_csv(coords, csv_path: str):
    os.makedirs(os.path.dirname(csv_path) or ".", exist_ok=True)
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["x", "y", "pen"])
        w.writerows(coords)
    print(f"✅ Saved TSP path CSV to: {csv_path}")


# -------------------------------------------------
# Visualization helper (optional, but super useful)
# -------------------------------------------------
def visualize_plotter_path(csv_path, output_path=None,
                           canvas_size=(800, 800), delay=1):
    """
    Visualize the generated TSP plotter path.
    pen=1 → red dots (pen up / travel)
    pen=2 → black line (drawing)
    """
    if not os.path.exists(csv_path):
        print(f"CSV not found: {csv_path}")
        return

    df = pd.read_csv(csv_path)
    xs, ys, pens = df["x"].to_numpy(), df["y"].to_numpy(), df["pen"].to_numpy()

    min_x, max_x = np.min(xs), np.max(xs)
    min_y, max_y = np.min(ys), np.max(ys)
    scale_x = (canvas_size[0] - 20) / (max_x - min_x + 1e-6)
    scale_y = (canvas_size[1] - 20) / (max_y - min_y + 1e-6)

    def to_canvas(x, y):
        cx = int((x - min_x) * scale_x + 10)
        cy = int(canvas_size[1] - ((y - min_y) * scale_y + 10))
        return cx, cy

    canvas = np.ones((canvas_size[1], canvas_size[0], 3), dtype=np.uint8) * 255
    prev_pt = None
    out = None

    if output_path:
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        out = cv2.VideoWriter(output_path, fourcc, 60, canvas_size)

    print("🎥 Visualizing TSP path (press Q to quit)...")

    for x, y, pen in zip(xs, ys, pens):
        pt = to_canvas(x, y)

        if pen == 2 and prev_pt is not None:
            cv2.line(canvas, prev_pt, pt, (0, 0, 0), 1)
        elif pen == 1:
            cv2.circle(canvas, pt, 2, (0, 0, 255), -1)

        prev_pt = pt

        cv2.imshow("TSP Plotter Path", canvas)
        if out is not None:
            out.write(canvas)
        if cv2.waitKey(delay) & 0xFF == ord("q"):
            break

    cv2.destroyAllWindows()
    if out is not None:
        out.release()
        print(f"🎞 Saved visualization video to: {output_path}")
    else:
        print("✅ Visualization complete (no video saved).")


# -------------------------------------------------
# Main
# -------------------------------------------------
if __name__ == "__main__":
    import sys
    import os

    if len(sys.argv) < 2:
        print("Usage: python tsp_from_skeleton.py <path_to_skeleton_image>")
        sys.exit(1)

    image_path = sys.argv[1]
    if not os.path.exists(image_path):
        print(f"❌ Image not found: {image_path}")
        sys.exit(1)

    # Output file names
    base = os.path.splitext(os.path.basename(image_path))[0]
    csv_path = f"{base}_tsp.csv"
    video_path = f"{base}_tsp.mp4"

    # Load skeletonized image (0=black, 255=white)
    skeleton = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if skeleton is None:
        print(f"❌ Could not read image: {image_path}")
        sys.exit(1)

    # Generate TSP path
    coords = skeleton_to_tsp_path(
        skeleton_img=skeleton,
        max_points=500,        # reduce if very large image
        jump_thresh=10.0,       # pixels — when to lift pen
    )

    # Save CSV
    save_coords_to_csv(coords, csv_path)

    # Optional visualization
    visualize_plotter_path(csv_path, output_path=video_path)

    print(f"\n✅ Done!\nCSV: {csv_path}\nVideo: {video_path}")
