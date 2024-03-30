import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from scipy.interpolate import CubicSpline

class ParametricPath:
    def __init__(self, path_function):
        self.path_function = path_function

    def __call__(self, t):
        return self.path_function(t)

def generate_gvf(path_function, kN):
    fig, ax = plt.subplots()
    
    # Plot path
    t_values = np.linspace(0, 2 * np.pi, 100)
    path_points = np.array([path_function(t) for t in t_values])
    ax.plot(path_points[:, 0], path_points[:, 1], 'g-', label='Path')

    # Plot tangent vectors and guiding vector field
    for t in np.linspace(0, 2 * np.pi, num=50):
        point = path_function(t)
        tangent = path_function(t + 0.01) - point  # Approximate tangent
        tangent /= np.linalg.norm(tangent)
        
        # Ideal direction vector
        ideal_direction = path_function(t + 0.01) - path_function(t - 0.01)
        ideal_direction /= np.linalg.norm(ideal_direction)
        
        # Calculate error as cosine of the angle between tangent and ideal direction
        error = np.dot(tangent, ideal_direction)
        
        # Calculate guiding vector
        gvf = tangent + kN * error * ideal_direction
        
        # Scale vectors to have fixed length
        scale_factor = 0.5  # Choose a scale factor for the vector length
        gvf_scaled = gvf * scale_factor
        
        ax.arrow(point[0], point[1], gvf_scaled[0], gvf_scaled[1], head_width=0.05, head_length=0.1, color='blue', alpha=0.5)

    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Guiding Vector Field')
    ax.legend()
    plt.show()

def circle_path(t):
    radius = 1.0
    x = radius * np.cos(t)
    y = radius * np.sin(t)
    return np.array([x, y])

# Define the points that the robot will pass through
points = np.array([[0, 0], [1, 2], [3, 4], [5, 1], [0, 0]])  # Adding the first point as the last point to close the loop

# Perform cubic spline interpolation to generate a smooth path
t_values = np.linspace(0, 1, len(points))
spline_x = CubicSpline(t_values, points[:, 0], bc_type='periodic')  
spline_y = CubicSpline(t_values, points[:, 1], bc_type='periodic')

def spline_path(t):
    return np.array([spline_x(t), spline_y(t)])

parametric_path_spline = ParametricPath(spline_path)

kN = 0
generate_gvf(parametric_path_spline, kN)
