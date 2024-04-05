import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

class ParametricPath:
    def __init__(self, path_function):
        self.path_function = path_function

    def __call__(self, t):
        return self.path_function(t)

def generate_gvf(path_function, kN, num_points=100):
    # parametric points
    t_values = np.linspace(0, 2 * np.pi, num_points)
    path_points = np.array([path_function(t) for t in t_values])
    
    # tangent vectors
    tangents = []
    for t in t_values:
        tangent = path_function(t + 0.01) - path_function(t - 0.01)
        tangents.append(tangent / np.linalg.norm(tangent))

    # ideal direction
    ideal_directions = np.roll(path_points, -1, axis=0) - np.roll(path_points, 1, axis=0)
    ideal_directions /= np.linalg.norm(ideal_directions, axis=1)[:, np.newaxis]

    # error function
    errors = np.sum(tangents * ideal_directions, axis=1)

    # gvf calculation (see paper)
    gvfs = tangents + kN * errors[:, np.newaxis] * ideal_directions

    # vector visualization
    scale_factor = 0.5
    gvfs_scaled = gvfs * scale_factor

    return path_points, gvfs_scaled

points = np.array([[0, 0], [1, 2], [3, 4], [5, 1], [0, 0]])

t_values = np.linspace(0, 1, len(points))
spline_x = CubicSpline(t_values, points[:, 0], bc_type='periodic')  
spline_y = CubicSpline(t_values, points[:, 1], bc_type='periodic')

def spline_path(t):
    return np.array([spline_x(t), spline_y(t)])

parametric_path_spline = ParametricPath(spline_path)

kN = 0.5
path_points, gvfs_scaled = generate_gvf(parametric_path_spline, kN)

plt.figure(figsize=(8, 6))
plt.plot(path_points[:, 0], path_points[:, 1], 'g-', label='Path')
plt.quiver(path_points[:, 0], path_points[:, 1], gvfs_scaled[:, 0], gvfs_scaled[:, 1], scale=20)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Guiding Vector Field with Convergence to Parametric Path')
plt.legend()
plt.grid(True)
plt.gca().set_aspect('equal', adjustable='box')
plt.show()