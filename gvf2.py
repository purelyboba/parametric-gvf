import numpy as np
import matplotlib.pyplot as plt

def normalize_vector_field(vx, vy):
    magnitude = np.sqrt(vx**2 + vy**2)
    return vx / magnitude, vy / magnitude

def gvf_gen(x, y, path_x, path_y):
    # Parameters
    tau = 1.0  # Magnitude of attraction to path
    k_ne = 0.5  # Magnitude of repulsion from obstacles
    delta = tau - k_ne

    # Compute distance to path
    dist_to_path = np.sqrt((x[:, :, np.newaxis] - path_x)**2 + (y[:, :, np.newaxis] - path_y)**2)

    # Compute direction of the path
    path_direction_x = np.gradient(path_x)
    path_direction_y = np.gradient(path_y)
    path_direction_magnitude = np.sqrt(path_direction_x**2 + path_direction_y**2)
    path_direction_x /= path_direction_magnitude
    path_direction_y /= path_direction_magnitude

    # Compute guiding vector components
    tau_x = tau * path_direction_x
    tau_y = tau * path_direction_y

    # Compute repulsion vector components (assuming no obstacles, for simplicity)
    k_ne_x = k_ne * (x[:, :, np.newaxis] - path_x) / dist_to_path
    k_ne_y = k_ne * (y[:, :, np.newaxis] - path_y) / dist_to_path

    # Compute guiding vector field
    v_x = tau_x - k_ne_x
    v_y = tau_y - k_ne_y

    return v_x.sum(axis=2), v_y.sum(axis=2)

# Define the path
path_x = np.linspace(0, 10, 100)
path_y = np.sin(path_x)

# Generate grid points
x = np.linspace(0, 10, 50)
y = np.linspace(-1, 1, 50)
X, Y = np.meshgrid(x, y)

# Compute the guiding vector field
Vx, Vy = gvf_gen(X, Y, path_x, path_y)
Vx, Vy = normalize_vector_field(Vx, Vy)

# Plotting
plt.figure(figsize=(8, 6))
plt.quiver(X, Y, Vx, Vy, scale=20)
plt.plot(path_x, path_y, 'r--', label='Path')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Guiding Vector Field with Convergence to Path')
plt.legend()
plt.grid(True)
plt.show()
