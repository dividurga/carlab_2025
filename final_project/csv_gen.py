import cv2
import numpy as np
from collections import deque
import csv
import math

# ---------------------------------------------------
# RDP Simplification
# ---------------------------------------------------
def rdp(points, epsilon):
    """Ramer–Douglas–Peucker simplification."""
    if len(points) < 3:
        return points

    start, end = points[0], points[-1]
    sx, sy = start
    ex, ey = end

    max_dist = -1
    index = -1

    for i, (x, y) in enumerate(points[1:-1], start=1):
        # perpendicular distance from line start->end
        if (ex - sx) == (ey - sy) == 0:
            dist = math.hypot(x - sx, y - sy)
        else:
            dist = abs((ey - sy)*x - (ex - sx)*y + ex*sy - ey*sx) / math.hypot(ex - sx, ey - sy)

        if dist > max_dist:
            max_dist = dist
            index = i

    if max_dist > epsilon:
        left = rdp(points[:index+1], epsilon)
        right = rdp(points[index:], epsilon)
        return left[:-1] + right
    else:
        return [start, end]


# ---------------------------------------------------
# Remove strokes too close to each other
# ---------------------------------------------------
def remove_close_strokes(paths, min_distance=100.0):
    kept = []

    def centroid(path):
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        return (sum(xs)/len(xs), sum(ys)/len(ys))

    centroids = [centroid(p) for p in paths]

    for i, path in enumerate(paths):
        cx, cy = centroids[i]
        too_close = False

        for ki in kept:
            kx, ky = centroids[ki]
            if math.hypot(cx - kx, cy - ky) < min_distance:
                too_close = True
                break

        if not too_close:
            kept.append(i)

    return [paths[i] for i in kept]


# ---------------------------------------------------
# Generate strokes from skeleton
# ---------------------------------------------------
def skeleton_to_paths(skel_img):
    H, W = skel_img.shape

    pixels = [(x, y) for y in range(H) for x in range(W) if skel_img[y, x] == 0]
    pixels_set = set(pixels)
    visited = set()

    neighbors = [(-1,-1),(0,-1),(1,-1),
                 (-1,0),        (1,0),
                 (-1,1),(0,1),(1,1)]

    paths = []

    def get_unvisited_neighbors(x, y):
        return [(nx, ny) for dx, dy in neighbors
                if (nx := x+dx, ny := y+dy) in pixels_set
                and (nx, ny) not in visited]

    while len(visited) < len(pixels):
        remaining = [p for p in pixels if p not in visited]
        start = min(remaining, key=lambda p: p[0]**2 + p[1]**2)
        path = [start]
        visited.add(start)
        current = start

        while True:
            nbs = get_unvisited_neighbors(*current)
            if not nbs:
                break
            nx, ny = min(nbs, key=lambda p: (p[0]-current[0])**2 +
                                         (p[1]-current[1])**2)
            visited.add((nx, ny))
            path.append((nx, ny))
            current = (nx, ny)

        paths.append(path)

    return paths


# ---------------------------------------------------
# TSP Reorder with Optional Reversal
# ---------------------------------------------------
def tsp_order_strokes(paths):
    if len(paths) <= 1:
        return paths

    strokes = []
    for i, p in enumerate(paths):
        strokes.append({
            "index": i,
            "start": p[0],
            "end": p[-1],
            "points": p
        })

    visited = set()
    ordered = []

    current = min(strokes, key=lambda s: s["start"][0]**2 + s["start"][1]**2)
    visited.add(current["index"])
    ordered.append(current)

    while len(visited) < len(strokes):
        best = None
        best_dist = 1e18
        best_reverse = False

        for s in strokes:
            if s["index"] in visited:
                continue

            d1 = math.hypot(current["end"][0] - s["start"][0],
                            current["end"][1] - s["start"][1])
            d2 = math.hypot(current["end"][0] - s["end"][0],
                            current["end"][1] - s["end"][1])

            if d1 < best_dist:
                best, best_dist, best_reverse = s, d1, False
            if d2 < best_dist:
                best, best_dist, best_reverse = s, d2, True

        if best_reverse:
            best["points"] = list(reversed(best["points"]))
            best["start"], best["end"] = best["end"], best["start"]

        visited.add(best["index"])
        ordered.append(best)
        current = best

    return [s["points"] for s in ordered]


# ---------------------------------------------------
# Save to CSV
# ---------------------------------------------------
def save_paths_to_csv(paths, csv_path):
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y"])
        for p in paths:
            for (x, y) in p:
                writer.writerow([x, y])
            writer.writerow([])


# ---------------------------------------------------
# Crop top/bottom
# ---------------------------------------------------
def remove_top_bottom(img, pct=0.10):
    H, W = img.shape
    cut = int(H * pct)
    img[:cut, :] = 255
    img[H-cut:, :] = 255
    return img


# ---------------------------------------------------
# Master pipeline
# ---------------------------------------------------
def extract_coords_from_skeleton_image(path_to_image, csv_path):

    img = cv2.imread(path_to_image, cv2.IMREAD_GRAYSCALE)
    _, bin_img = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)

    bin_img = remove_top_bottom(bin_img)

    # 1. Generate raw strokes
    raw_paths = skeleton_to_paths(bin_img)

    # 2. Remove strokes too close to each other
    filtered_paths = remove_close_strokes(raw_paths, min_distance=2)

    # 3. Simplify each stroke
    simplified = [rdp(p, epsilon=2.0) for p in filtered_paths]

    # 4. TSP reorder for optimal drawing
    tsp_paths = tsp_order_strokes(simplified)

    # 5. Save
    save_paths_to_csv(tsp_paths, csv_path)

    return tsp_paths


if __name__ == "__main__":
    extract_coords_from_skeleton_image("captured_line.png", "coords.csv")
