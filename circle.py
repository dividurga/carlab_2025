import math

def generate_circle_points(center=(15, 15), radius=15.0, n_points=20):
    cx, cy = center
    pts = []

    for i in range(n_points):
        theta = 2 * math.pi * i / n_points
        x = cx + radius * math.cos(theta)
        y = cy + radius * math.sin(theta)
        pts.append((x, y))

    return pts


def main():
    center = (25, 25)
    radius = 15
    n = 20

    print(f"Circle: (x - {center[0]})^2 + (y - {center[1]})^2 = {radius**2}\n")

    pts = generate_circle_points(center, radius, n)

    print("Coordinates:")
    for x, y in pts:
        print(f"{x:.3f}, {y:.3f}")


if __name__ == "__main__":
    main()
