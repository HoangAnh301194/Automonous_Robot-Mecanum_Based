import numpy as np

thetas = np.array([-np.pi, 0, np.pi])
angle_min = -np.pi
num_bins = 720
angle_increment = 2 * np.pi / num_bins

bin_idx = ((thetas - angle_min) / angle_increment).astype(np.int32)
# Ensure bin_idx wraps properly for +pi
bin_idx[bin_idx >= num_bins] = num_bins - 1

print(f"Bin idx: {bin_idx}")
