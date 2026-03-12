import csv
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

CELLS = 144
MID = CELLS // 2  # 72

def read_points_csv(path):
    pts = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            pts.append((float(row["x"]), float(row["y"])))
    return pts

def read_boxes_csv(path):
    boxes = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            boxes.append((float(row["x"]), float(row["y"]), float(row["w"]), float(row["h"])))
    return boxes

def rc_to_xy(r, c):
    # x right +, y up +, centered at MID
    x = c - MID
    y = MID - r
    return x, y

# --------------------------
# Read CSVs
# --------------------------
path_pts = read_points_csv("Plot/path.csv")
blocked_pts = read_points_csv("Plot/blocked.csv")
penalty_pts = read_points_csv("Plot/penalty.csv")

# Optional: only if you generated robot_boxes.csv
try:
    robot_boxes = read_boxes_csv("Plot/robot_boxes.csv")
except FileNotFoundError:
    robot_boxes = []

# --------------------------
# Convert to (x,y)
# --------------------------
path_xy = [rc_to_xy(r, c) for (r, c) in path_pts]
blk_xy  = [rc_to_xy(r, c) for (r, c) in blocked_pts]
pen_xy  = [rc_to_xy(r, c) for (r, c) in penalty_pts]

path_x = [x for x, y in path_xy]
path_y = [y for x, y in path_xy]

blk_x = [x for x, y in blk_xy]
blk_y = [y for x, y in blk_xy]

pen_x = [x for x, y in pen_xy]
pen_y = [y for x, y in pen_xy]

plt.figure()
ax = plt.gca()

# --------------------------
# Draw robot footprint rectangles (in xy)
# --------------------------
for (r, c, w, h) in robot_boxes:
    x, y = rc_to_xy(r, c)

    # Rectangle wants bottom-left corner
    left = x - (w / 2.0)
    bottom = y - (h / 2.0)

    rect = Rectangle((left, bottom), w, h, fill=False, linewidth=1.2, label="_robot")
    ax.add_patch(rect)

# --------------------------
# Plot blocked / penalty / path
# --------------------------
if blocked_pts:
    plt.scatter(blk_x, blk_y, s=8, label="blocked")

if penalty_pts:
    plt.scatter(pen_x, pen_y, s=12, marker="s", label="penalty")

if path_pts:
    plt.plot(path_x, path_y, linewidth=2, label="path")

    # Mark start and goal from path endpoints
    plt.scatter([path_x[0]], [path_y[0]], s=60, marker="o", label="start")
    plt.scatter([path_x[-1]], [path_y[-1]], s=60, marker="x", label="goal")
else:
    print("No path points found (path.csv empty).")

# --------------------------
# Make axes match centered plane coords
# --------------------------
plt.gca().set_aspect("equal", adjustable="box")

# Full field visible in centered coords:
# c=0..143 -> x = -72..71
# r=0..143 -> y = 72..-71
plt.xlim(-MID, MID - 1)
plt.ylim(-(MID - 1), MID)

plt.title("A* path on centered plane coords (x,y)")
plt.xlabel("x (inches from center)")
plt.ylabel("y (inches from center)")
plt.legend()
plt.show()
