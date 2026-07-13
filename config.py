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
NOISE_STD = 1.0 
NUM_SENSORS = 6
RAYS_PER_SENSOR = 15
SENSOR_ANGLES_DEG = [0, 60, 120, 180, 240, 300] # Mounting angles on the frame

# Generate individual beam offset angles for each distance sensor
ray_angles_deg = np.linspace(-32, 32, RAYS_PER_SENSOR).tolist()

# === PRE-COMPUTED RADIANS (avoids math.radians(math.degrees(...)) hot loop) ===
SENSOR_ANGLES_RAD = [math.radians(a) for a in SENSOR_ANGLES_DEG]
ray_angles_rad = [math.radians(a) for a in ray_angles_deg]

# ============================================================
# PHYSICS & DYNAMICS PARAMETERS (Velocity Ramping Model)
# ============================================================
ROBOT_WHEELBASE = 8.0      # turning responsiveness (lower = faster turns)
MAX_SPEED = 3.0            # max per-tick displacement at cmd=1.0
ACCEL_RATE = 0.25          # per-tick approach to target velocity (acceleration)
DECEL_RATE = 0.45          # per-tick braking when stopping or reversing
STOP_THRESHOLD = 0.01      # velocity below which we snap to zero

# Noisy odometry (set to zero for deterministic navigation)
ODOMETRY_NOISE_STD = 0.0   # was 0.3 — zeroed to prevent wall hits from drift
GYRO_DRIFT_STD = 0.0       # was 0.015
ODOMETRY_DRIFT_RATE = 0.0  # was 0.02

# Noisy odometry
ODOMETRY_NOISE_STD = 0.3   # std dev of additive noise per tick
GYRO_DRIFT_STD = 0.015     # std dev of heading drift per tick
ODOMETRY_DRIFT_RATE = 0.02 # systematic drift per unit distance traveled