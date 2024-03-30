import numpy as np
import matplotlib.pyplot as plt
import tkinter as tk
from tkinter import messagebox
from scipy.interpolate import CubicSpline

class ParametricPath:
    def __init__(self, points):
        t_values = np.linspace(0, 1, len(points))
        self.spline_x = CubicSpline(t_values, points[:, 0])
        self.spline_y = CubicSpline(t_values, points[:, 1])

    def __call__(self, t):
        return np.array([self.spline_x(t), self.spline_y(t)])

class PathEditor:
    def __init__(self, master):
        self.master = master
        self.master.title("Parametric Path Editor")

        self.points = []
        self.canvas = tk.Canvas(self.master, width=400, height=400, bg='white')
        self.canvas.pack()

        self.canvas.bind("<Button-1>", self.add_point)
        self.generate_button = tk.Button(self.master, text="Generate Path", command=self.generate_path)
        self.generate_button.pack()

    def add_point(self, event):
        x, y = event.x, event.y
        canvas_height = self.canvas.winfo_height()
        self.points.append([x, canvas_height - y])
        self.canvas.create_oval(x-3, y-3, x+3, y+3, fill='black')


    def generate_path(self):
        if len(self.points) < 2:
            messagebox.showerror("Error", "Please add at least two points.")
            return
        
        points_array = np.array(self.points)
        parametric_path = ParametricPath(points_array)

        fig, ax = plt.subplots()
        t_values = np.linspace(0, 1, 100)
        path_points = np.array([parametric_path(t) for t in t_values])
        ax.plot(path_points[:, 0], path_points[:, 1], 'g-', label='Path')
        
        # Plot the waypoints
        ax.scatter(points_array[:, 0], points_array[:, 1], color='red', label='Waypoints')
        
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_title('Generated Path')
        ax.legend()
        plt.show()

def main():
    root = tk.Tk()
    app = PathEditor(root)
    root.mainloop()

if __name__ == "__main__":
    main()
