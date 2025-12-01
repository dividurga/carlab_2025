import math

def generate_left_arc_points(center=(15, 15), radius=15.0, n_points=20):
    cx, cy = center
    pts = []

    # Start at bottom: theta = 3π/2
    # End at top-left: theta = π
    theta_start = 3 * math.pi / 2
    theta_end   = math.pi / 2

    for i in range(n_points):
        theta = theta_start + (theta_end - theta_start) * (i / (n_points - 1))
        x = cx + radius * math.cos(theta)
        y = cy + radius * math.sin(theta)
        pts.append((x, y))

    return pts


def main():
    center = (52, 25)
    radius = math.sqrt(200)
    n = 20

    print(f"Left arc: bottom → top-left\n")

    pts = generate_left_arc_points(center, radius, n)

    print("Coordinates:")
    for x, y in pts:
        print(f"{x:.2f}")
    print("\nY values:")
    for x, y in pts:
        print(f"{y:.2f}")


if __name__ == "__main__":
    main()
