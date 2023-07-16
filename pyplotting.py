import os

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from mpl_toolkits.mplot3d import Axes3D

matplotlib.use("TkAgg")
directory = os.getcwd()  # Get the current working directory
files = [
    file for file in os.listdir(directory) if file.startswith("output")
]  # Get a list of files matching the pattern

num_sats = len(files)  # Count the number of files

# Specify file level properties
data_lines = [2, None]
delimiter = ","
column_names = ["x", "y", "z"]
column_types = [float, float, float]

# Create a new figure
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

# Plot the data
for i in range(num_sats):
    fn = "output" + str(i)
    filename = fn + ".txt"
    output = pd.read_csv(
        filename,
        skiprows=data_lines[0] - 1,
        delimiter=delimiter,
        names=column_names,
        dtype=dict(zip(column_names, column_types)),
    )

    satname = "sat" + str(i)
    x = np.array(output["x"])  # Convert to NumPy array
    y = np.array(output["y"])  # Convert to NumPy array
    z = np.array(output["z"])  # Convert to NumPy array

    ax.plot3D(x, y, z, label=satname)

# Set plot limits and labels
ax.set_xlim([-10000e3, 10000e3])
ax.set_ylim([-10000e3, 10000e3])
ax.set_zlim([-10000e3, 10000e3])
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")
ax.set_box_aspect([1.0, 1.0, 1.0])
ax.grid(True)

# Add legend
ax.legend()

# Display the plot
plt.show()
