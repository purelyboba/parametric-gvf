import numpy as np
import matplotlib.pyplot as plt
import tkinter as tk
from tkinter import messagebox
from scipy.interpolate import CubicSpline, PchipInterpolator

class ParametricPath:
    def __init__(self, points):
        t_values = np.linspace(0, 1, len(points))
        # cubic hermite interpolation
        self.spline_x = PchipInterpolator(t_values, points[:, 0])
        self.spline_y = PchipInterpolator(t_values, points[:, 1])

    def __call__(self, t):
        return np.array([self.spline_x(t), self.spline_y(t)])

class PathEditor:
    def __init__(self, master):
        self.master = master
        self.master.title("Parametric Path Editor")

        self.points = []
        self.canvas = tk.Canvas(self.master, width=1000, height=1000, bg='white')
        self.canvas.pack()

        self.bg_image = tk.PhotoImage(file="bg.png")
        self.canvas.create_image(0, 0, anchor=tk.NW, image=self.bg_image)

        self.canvas.bind("<Button-1>", self.add_point)
        self.generate_button = tk.Button(self.master, text="Generate Path", command=self.generate_path)
        self.generate_button.pack()

    def add_point(self, event):
        x, y = event.x, event.y

        canvas_height = self.canvas.winfo_height()
        self.points.append([x, canvas_height - y])
        self.canvas.create_oval(x-3, y-3, x+3, y+3, fill='black')

    def generate_gvf(self, path_function, kN, num_points=300):
        print(num_points)

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

        return path_points, gvfs

    def generate_path(self):
        if len(self.points) < 2:
            messagebox.showerror("Error", "Please add at least two points.")
            return
        
        points_array = np.array(self.points)
        parametric_path = ParametricPath(points_array)

        kN = 0.5
        gvfpath_points, gvfs = self.generate_gvf(parametric_path, kN)

        fig = plt.figure(figsize=(8, 6))
        t_values = np.linspace(0, 1, 1000)
        path_points = np.array([parametric_path(t) for t in t_values])
        plt.plot(path_points[:, 0], path_points[:, 1], 'g-', label='Path')

        plt.scatter(points_array[:, 0], points_array[:, 1], color='red', label='Waypoints')
        plt.quiver(gvfpath_points[:, 0], gvfpath_points[:, 1], gvfs[:, 0], gvfs[:, 1])
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Generated Path')

        plt.xlim(0, 1000)

        plt.ylim(0, 1000)
        plt.legend()
        plt.show()

def main():
    root = tk.Tk()
    app = PathEditor(root)
    root.mainloop()

if __name__ == "__main__":
    main()
