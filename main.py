'''
main.py

Entry point for the Pure Pursuit path tracking simulation.
Runs the simulation loop and coordinates the vehicle model,
controller, and path generator.
'''

# Import required libraries

import numpy as np
from config import *
from models.bicycle_model import BicycleModel
from controller.pure_pursuit import PurePursuitController
from trajectory.path_generator import generate_circle, generate_straight_line
from visualization.plotter import plot_simulation

# Initialize
# path = generate_straight_line(100.0, NUM_POINTS)
# vehicle = BicycleModel(0.0, 3.0, 0.0, 5.0, WHEELBASE)
path = generate_circle(CIRCLE_RADIUS, NUM_POINTS)
vehicle = BicycleModel(CIRCLE_RADIUS - 3.0, 0.0, np.pi/2, 5.0, WHEELBASE)
controller = PurePursuitController(LOOKAHEAD_DISTANCE, WHEELBASE)
history = []

t = 0.0
while t < SIM_TIME:
    # Get current vehicle state
    state       = vehicle.get_state()
    vehicle_pos = state[:2]

    # Compute control
    lookahead_point = controller.find_lookahead_point(vehicle_pos, path)
    delta           = controller.compute_steering(vehicle_pos, state[2], lookahead_point)
    delta           = np.clip(delta, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE)

    # Update and log
    vehicle.update(delta, DT)
    history.append(vehicle_pos.copy())

    t += DT

    # Check goal
    if t > 5.0 and np.linalg.norm(vehicle_pos - path[-1]) < GOAL_THRESHOLD:
        break

print("Simulation finished")
plot_simulation(path, history)