import csv
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


def read_points_csv(path):
    pts = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            pts.append((int(row["r"]), int(row["c"])))
    return pts

def read_boxes_csv(path):
    boxes = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            boxes.append((int(row["r"]), int(row["c"]), int(row["w"]), int(row["h"])))
    return boxes


# --------------------------
# Read exported CSVs
# --------------------------
path_pts = read_points_csv("path.csv")
blocked_pts = read_points_csv("blocked.csv")
robot_boxes = read_boxes_csv("robot_boxes.csv")


# penalty.csv is new (Park Zones)
# If you run before generating penalty.csv, you’ll get an error.
penalty_pts = read_points_csv("penalty.csv")

# --------------------------
# Convert (r,c) -> (x,y) for plotting
# x = c (horizontal), y = r (vertical)
# --------------------------
path_x = [c for r, c in path_pts]
path_y = [r for r, c in path_pts]

blk_x = [c for r, c in blocked_pts]
blk_y = [r for r, c in blocked_pts]

pen_x = [c for r, c in penalty_pts]
pen_y = [r for r, c in penalty_pts]

plt.figure()

ax = plt.gca()

# Draw robot footprint rectangles
# (r,c) is center in your grid; rectangle needs bottom-left corner
for (r, c, w, h) in robot_boxes:
    hw = w / 2.0
    hh = h / 2.0

    # Rectangle expects (x,y) = (left, top) in data coords,
    # but since we invert_yaxis later, we can still place it using grid coords.
    left = c - hw
    top  = r - hh

    rect = Rectangle((left, top), w, h, fill=False, linewidth=1.2, label="_robot")
    ax.add_patch(rect)

# Blocked cells (walls, goals, etc.)
if blocked_pts:
    plt.scatter(blk_x, blk_y, s=8, label="blocked")

# Penalty cells (Park Zones) -> different marker so you can see them
if penalty_pts:
    plt.scatter(pen_x, pen_y, s=12, marker="s", label="penalty")

# Path line
if path_pts:
    plt.plot(path_x, path_y, linewidth=2, label="path")

    # Mark start and goal based on path endpoints
    plt.scatter([path_x[0]], [path_y[0]], s=60, marker="o", label="start")
    plt.scatter([path_x[-1]], [path_y[-1]], s=60, marker="x", label="goal")
else:
    print("No path points found (path.csv empty).")

plt.gca().set_aspect("equal", adjustable="box")
plt.gca().invert_yaxis()  # row 0 at top like a grid
plt.title("A* path on grid (blocked + penalty)")
plt.xlabel("c (column)")
plt.ylabel("r (row)")
plt.legend()
plt.show()
