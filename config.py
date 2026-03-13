"""
Configuration parameters for Pure Pursuit Simulation.

All tunable parameters in one place.
"""

import math

# ============================================================
# Vehicle Parameters
# ============================================================
WHEELBASE = 2.5                         # [m] Distance between front and rear axle (L)
MAX_STEERING_ANGLE = math.radians(35)   # [rad] ~0.61 rad. Max steering angle
MAX_SPEED = 10.0                        # [m/s] Maximum vehicle speed

# ============================================================
# Pure Pursuit Parameters
# ============================================================
LOOKAHEAD_DISTANCE = 5.0    # [m] Base lookahead distance (Ld)
MIN_LOOKAHEAD = 2.0         # [m] Minimum lookahead distance
MAX_LOOKAHEAD = 15.0        # [m] Maximum lookahead distance
KDD = 0.5                   # Lookahead gain for adaptive Ld (Ld = KDD * v)

# ============================================================
# Simulation Parameters
# ============================================================
DT = 0.01                   # [s] Time step for Euler integration
SIM_TIME = 60.0             # [s] Total simulation time
GOAL_THRESHOLD = 1.0        # [m] Distance to consider goal reached

# ============================================================
# Trajectory Parameters
# ============================================================
CIRCLE_RADIUS = 20.0        # [m] Radius for circular trajectory
NUM_POINTS = 500            # Number of waypoints in trajectory

# ============================================================
# Visualization Parameters
# ============================================================
ANIMATION_INTERVAL = 20     # [ms] Time between animation frames
PLOT_EVERY_N = 5            # Plot every N simulation steps (performance)