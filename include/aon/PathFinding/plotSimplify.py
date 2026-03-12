import matplotlib.pyplot as plt
import csv

def read_points(filename):
    xs = []
    ys = []

    with open(filename) as f:
        reader = csv.reader(f)
        for row in reader:
            if row[0] == 'y' or row[1] == 'x':
                continue
            xs.append(float(row[0]))
            ys.append(float(row[1]))

    return xs, ys


raw_x, raw_y = read_points("Plot/path.csv")
rdp_x, rdp_y = read_points("Plot/rdp_path.csv")
spline_x, spline_y = read_points("Plot/spline_path.csv")
even_x, even_y = read_points("Plot/even_path.csv")


fig, ax = plt.subplots(2,2, figsize=(10,10))


# RAW PATH
ax[0,0].plot(raw_x, raw_y, 'o-', label="Raw Path")
ax[0,0].set_title("Raw Path")
ax[0,0].axis("equal")
ax[0,0].grid()


# RDP
ax[0,1].plot(raw_x, raw_y, '--', label="Original")
ax[0,1].plot(rdp_x, rdp_y, 'o-', label="RDP")
ax[0,1].set_title("RDP Simplification")
ax[0,1].axis("equal")
ax[0,1].legend()
ax[0,1].grid()


# SPLINE
ax[1,0].plot(rdp_x, rdp_y, 'o', label="Control Points")
ax[1,0].plot(spline_x, spline_y, '-', label="Catmull-Rom")
ax[1,0].set_title("Spline")
ax[1,0].axis("equal")
ax[1,0].legend()
ax[1,0].grid()


# EVEN SAMPLING
ax[1,1].plot(spline_x, spline_y, '-', label="Spline")
ax[1,1].plot(even_x, even_y, 'o', label="Even Samples")
ax[1,1].set_title("Arc Length Sampling")
ax[1,1].axis("equal")
ax[1,1].legend()
ax[1,1].grid()


plt.tight_layout()
plt.show()