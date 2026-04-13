'''
plotter.py

Visualization utilities for the Pure Pursuit simulation.
'''

# Import required libraries
import numpy as np
import matplotlib.pyplot as plt

def plot_simulation(path: np.ndarray, history: list) -> None:
    '''
    Plots the reference path and the vehicle trajectory.

    Args:
        path:    Reference path as array of shape (N, 2).
        history: List of [x, y] positions recorded during simulation.
    '''
    trajectory = np.array(history)

    plt.figure(figsize=(8, 8))
    plt.plot(trajectory[:, 0], trajectory[:, 1], color='blue',  linewidth=1.5,  label='Vehicle trajectory')
    plt.plot(path[:, 0],       path[:, 1],       color='gray',  linestyle='--', label='Reference path', linewidth=1.0)
    plt.scatter(*path[0],  color='green', marker='o', zorder=5, label='Start')
    plt.scatter(*path[-1], color='red',   marker='x', zorder=5, label='Goal')

    plt.title('Pure Pursuit Path Tracking')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    plt.tight_layout()
    plt.show()