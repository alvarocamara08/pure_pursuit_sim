'''
pure_pursuit.py

Pure Pursuit controller for path tracking on a kinematic bicycle model.
Computes the steering angle required to follow a reference path.
'''

# Import required libraries
import numpy as np

class PurePursuitController:
    '''
    Implements the Pure Pursuit path tracking algorithm.
    
    The controller selects a lookahead point on the reference path and
    computes the steering angle needed to follow a circular arc toward it.
    '''

    def __init__(self, lookahead_distance: float, wheelbase: float):
        self.L    = lookahead_distance  # lookahead distance [m]
        self.L_wb = wheelbase           # vehicle wheelbase [m]
        
    def find_lookahead_point(self, vehicle_pos: np.ndarray, path: np.ndarray) -> np.ndarray:
        '''
        Finds the lookahead point on the path.

        Locates the closest path point to the vehicle, then advances along
        the path to find the first point at least L meters away.

        Args:
            vehicle_pos: Current vehicle position as [x, y].
            path:        Reference path as array of shape (N, 2).

        Returns:
            Lookahead point as [x, y]. Returns the last path point if none
            is found at the required distance.
        '''
        min_dist    = np.inf
        closest_idx = 0

        for i, point in enumerate(path):
            dist = np.linalg.norm(point - vehicle_pos)
            if dist < min_dist:
                min_dist    = dist
                closest_idx = i

        for point in path[closest_idx:]:
            dist = np.linalg.norm(point - vehicle_pos)
            if dist >= self.L:
                return point

        return path[-1]
    
    def compute_steering(self, vehicle_pos: np.ndarray, vehicle_heading: float, 
                         lookahead_point: np.ndarray) -> float:
        '''
        Computes the steering angle using the Pure Pursuit algorithm.

        Transforms the lookahead point to the vehicle's local frame, computes
        the heading error alpha, and returns the required steering angle.

        Args:
            vehicle_pos:     Current vehicle position as [x, y].
            vehicle_heading: Current vehicle heading [rad].
            lookahead_point: Target lookahead point as [x, y].

        Returns:
            Steering angle delta [rad].
        '''
        dx = lookahead_point[0] - vehicle_pos[0]
        dy = lookahead_point[1] - vehicle_pos[1]

        x_local =  dx * np.cos(vehicle_heading) + dy * np.sin(vehicle_heading)
        y_local = -dx * np.sin(vehicle_heading) + dy * np.cos(vehicle_heading)
        alpha = np.arctan2(y_local, x_local)
        
        delta = np.arctan2(2 * self.L_wb * np.sin(alpha), self.L)

        return delta