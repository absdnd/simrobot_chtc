import matplotlib.pyplot as plt
import numpy as np

# Plot the trajectory from the trajectory path #  
traj_path = "data/trajectories/"
file_name = os.listdir(traj_path)[0]

with open(traj_path + file_name, "r") as f:
    lines = f.readlines()

lines = [line.strip() for line in lines]
lines = [line.split(",") for line in lines]
lines = [[float(val) for val in line] for line in lines]

lines = np.array(lines)
plt.plot(lines[:, 0], lines[:, 1])
