import csv
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ---------------------------------------------------------
# Load CSV with blank lines → strokes
# ---------------------------------------------------------
def load_strokes(csv_path):
    """
    Reads CSV of x,y where blank lines separate strokes.
    Returns: list of strokes; each stroke = list of (x,y)
    """
    strokes = []
    current = []

    with open(csv_path, "r") as f:
        reader = csv.reader(f)
        next(reader)  # skip header

        for row in reader:
            # blank line = stroke break
            if len(row) < 2 or row[0] == "" or row[1] == "":
                if current:
                    strokes.append(current)
                    current = []
                continue

            try:
                x, y = float(row[0]), float(row[1])
                current.append((x, y))
            except:
                pass

    if current:
        strokes.append(current)

    return strokes


# ---------------------------------------------------------
# Animate strokes with pen-up between them
# ---------------------------------------------------------
def animate_strokes(strokes, interval_ms=100, save_path=None):
    # Flatten for axis limits
    all_pts = [p for s in strokes for p in s]
    xs_all = [p[0] for p in all_pts]
    ys_all = [p[1] for p in all_pts]

    fig, ax = plt.subplots(figsize=(6, 8))
    ax.set_xlim(min(xs_all)-10, max(xs_all)+10)
    ax.set_ylim(max(ys_all)+10, min(ys_all)-10)  # invert Y-axis
    ax.set_aspect("equal")
    ax.set_title("Pen-Up Animation from CSV")

    # Make a line object for each stroke
    line_objs = []
    for _ in strokes:
        line, = ax.plot([], [], "k-", linewidth=1)
        line_objs.append(line)

    # Red current point
    dot, = ax.plot([], [], "ro", markersize=4)

    # Build list of frames
    frame_map = []
    for si, stroke in enumerate(strokes):
        for pi in range(len(stroke)):
            frame_map.append((si, pi))

    def init():
        for line in line_objs:
            line.set_data([], [])
        dot.set_data([], [])
        return line_objs + [dot]

    def update(frame):
        si, pi = frame_map[frame]

        # Draw strokes completed so far
        for idx, stroke in enumerate(strokes):
            if idx < si:
                xs = [p[0] for p in stroke]
                ys = [p[1] for p in stroke]
                line_objs[idx].set_data(xs, ys)
            elif idx == si:
                xs = [p[0] for p in stroke[:pi+1]]
                ys = [p[1] for p in stroke[:pi+1]]
                line_objs[idx].set_data(xs, ys)
            else:
                line_objs[idx].set_data([], [])

        x, y = strokes[si][pi]
        dot.set_data([x], [y])

        return line_objs + [dot]

    ani = animation.FuncAnimation(
        fig,
        update,
        init_func=init,
        frames=len(frame_map),
        interval=interval_ms,
        blit=True
    )

    # ---------------------------------------------
    # 🚀 SAVE ANIMATION IF save_path IS SPECIFIED
    # ---------------------------------------------
    if save_path is not None:
        print(f"Saving animation to {save_path}...")
        try:
            writer = animation.FFMpegWriter(fps=int(1000/interval_ms))
            ani.save(save_path, writer=writer)
            print("✔ Video saved successfully!")
        except Exception as e:
            print("⚠ Could not save video. Install ffmpeg:")
            print("  brew install ffmpeg  (macOS)")
            print("  sudo apt install ffmpeg  (Linux)")
            print(f"Error: {e}")

    plt.show()


# ---------------------------------------------------------
# Main
# ---------------------------------------------------------
if __name__ == "__main__":
    strokes = load_strokes("coords.csv")
    animate_strokes(strokes, 100, "visualized_csv.mp4")
