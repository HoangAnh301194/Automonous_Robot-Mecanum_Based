import numpy as np

fov = 1.3962634
w = 640
angle_min = -fov / 2.0
angle_increment = fov / w

# simulate rear camera
thetas = np.array([np.pi - 0.1, np.pi, np.pi + 0.1])
bin_idx = ((thetas - angle_min) / angle_increment).astype(np.int32)
in_bounds = (bin_idx >= 0) & (bin_idx < w)

print(f"Angle min: {angle_min}")
print(f"Thetas: {thetas}")
print(f"Bin idx: {bin_idx}")
print(f"In bounds: {in_bounds}")
