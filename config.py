import numpy as np
import math

# ============================================================
# MAZE ENVIRONMENT CONSTANTS
# ============================================================
N, M = 9, 10        # Maze dimensions (height, width)
CELL_SIZE = 30     # Size of a single grid cell in pixels/units
WALL_THICK = 2     # Thickness of the maze walls
PATH_WIDTH = CELL_SIZE - WALL_THICK

# ============================================================
# ROBOT GEOMETRY & DIMENSIONS (MECANUM)
# ============================================================
ROBOT_W_WIDTH = 10.0   # Robot width (lateral / strafe axis)
ROBOT_L_LENGTH = 10.0  # Robot length (longitudinal / forward-back axis)

# Sensor placement radius aligns with the longest edge of the robot rectangle
SENSOR_RADIUS = ROBOT_L_LENGTH / 2.0 

# ============================================================
# LASER SENSOR PARAMETERS (LIDAR / DISTANCE)
# ============================================================
SENSOR_RANGE = 200
NOISE_STD = 0.8 
NUM_SENSORS = 6
RAYS_PER_SENSOR = 15
SENSOR_ANGLES_DEG = [0, 60, 120, 180, 240, 300] # Mounting angles on the frame

# Generate individual beam offset angles for each distance sensor
ray_angles_deg = np.linspace(-32, 32, RAYS_PER_SENSOR).tolist()

# === PRE-COMPUTED RADIANS (avoids math.radians(math.degrees(...)) hot loop) ===
SENSOR_ANGLES_RAD = [math.radians(a) for a in SENSOR_ANGLES_DEG]
ray_angles_rad = [math.radians(a) for a in ray_angles_deg]

# ============================================================
# ROBOT CONTROL PARAMETERS
# ============================================================
ROBOT_SPEED_SCALE = 3.0    # simple velocity multiplier (servo-style, no inertia)