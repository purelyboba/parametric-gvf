import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Arrow, Circle
from matplotlib.animation import FuncAnimation

class RobotSimulation:
    def __init__(self):
        self.dt = 0.1  # delta t
        self.goal_x = 8.0
        self.goal_y = 8.0
        self.k = 1.0  # vf strength
        
        self.robot_state = {
            'x': 1.0,
            'y': 1.0,
            'theta': 0.0
        }
        
        # control
        self.K_p = 2.0  # p for steering
        self.v_max = 1.0  # max vel
        
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.setup_plot()
        
    def setup_plot(self):
        self.ax.set_xlim(-2, 10)
        self.ax.set_ylim(-2, 10)
        self.ax.grid(True)
        self.ax.set_aspect('equal')
        
        # vector field
        X, Y = np.meshgrid(np.linspace(-2, 10, 20), np.linspace(-2, 10, 20))
        U, V = np.zeros_like(X), np.zeros_like(Y)
        
        for i in range(X.shape[0]):
            for j in range(X.shape[1]):
                dx, dy = self.vector_field(X[i,j], Y[i,j])
                U[i,j] = dx
                V[i,j] = dy
        
        self.ax.quiver(X, Y, U, V, alpha=0.3)
        
        goal = Circle((self.goal_x, self.goal_y), 0.3, color='green', fill=True)
        self.ax.add_patch(goal)
        
        self.robot_circle = Circle((self.robot_state['x'], self.robot_state['y']), 
                                 0.3, color='blue', fill=False)
        self.direction_arrow = Arrow(self.robot_state['x'], self.robot_state['y'], 
                                   0.5*np.cos(self.robot_state['theta']), 
                                   0.5*np.sin(self.robot_state['theta']), 
                                   width=0.3, color='red')
        self.ax.add_patch(self.robot_circle)
        self.ax.add_patch(self.direction_arrow)
        
        self.path_x = [self.robot_state['x']]
        self.path_y = [self.robot_state['y']]
        self.path_line, = self.ax.plot(self.path_x, self.path_y, 'b--', alpha=0.5)
    
    # edit this for desired vf
    def vector_field(self, x, y):
        dx = self.goal_x - x
        dy = self.goal_y - y
        magnitude = np.sqrt(dx**2 + dy**2)
        if magnitude > 0:
            dx = self.k * dx / magnitude
            dy = self.k * dy / magnitude
        return dx, dy
    
    def compute_control(self):
        dx, dy = self.vector_field(self.robot_state['x'], self.robot_state['y'])
        desired_theta = np.arctan2(dy, dx)
        
        theta_error = np.arctan2(np.sin(desired_theta - self.robot_state['theta']),
                                np.cos(desired_theta - self.robot_state['theta']))
        
        # controls
        omega = self.K_p * theta_error
        v = self.v_max * np.cos(theta_error)
        
        return v, omega
    
    def update(self, frame):
        v, omega = self.compute_control()
        
        self.robot_state['x'] += v * np.cos(self.robot_state['theta']) * self.dt
        self.robot_state['y'] += v * np.sin(self.robot_state['theta']) * self.dt
        self.robot_state['theta'] += omega * self.dt
        
        self.robot_circle.center = (self.robot_state['x'], self.robot_state['y'])
        
        self.direction_arrow.remove()
        self.direction_arrow = Arrow(self.robot_state['x'], self.robot_state['y'],
                                   0.5*np.cos(self.robot_state['theta']),
                                   0.5*np.sin(self.robot_state['theta']),
                                   width=0.3, color='red')
        self.ax.add_patch(self.direction_arrow)
        
        self.path_x.append(self.robot_state['x'])
        self.path_y.append(self.robot_state['y'])
        self.path_line.set_data(self.path_x, self.path_y)
        
        return self.robot_circle, self.direction_arrow, self.path_line
    
    def run(self):
        anim = FuncAnimation(self.fig, self.update, frames=200, 
                           interval=50, blit=True)
        plt.show()

if __name__ == "__main__":
    sim = RobotSimulation()
    sim.run()