import random
import sys
import math
import time
import json
import hmac
import hashlib
import threading
import asyncio
import numpy as np
import websockets
import os
from dotenv import load_dotenv

load_dotenv()

# Import central hardware and environment settings
from config import (
    N, M, CELL_SIZE, WALL_THICK, PATH_WIDTH,
    ROBOT_W_WIDTH, ROBOT_L_LENGTH, SENSOR_RADIUS, 
    SENSOR_RANGE, NOISE_STD, NUM_SENSORS, 
    RAYS_PER_SENSOR, SENSOR_ANGLES_DEG, ray_angles_deg
)

def generate_maze_no_loops(width, height):
    """Generates a perfect logic maze grid using Depth-First Search."""
    maze = [[1] * (2 * width + 1) for _ in range(2 * height + 1)]
    directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]
    def walk(x, y):
        maze[2*y+1][2*x+1] = 0
        random.shuffle(directions)
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height and maze[2*ny+1][2*nx+1] == 1:
                maze[2*y+1+dy][2*x+1+dx] = 0
                walk(nx, ny)
    walk(0, 0)
    maze[1][1] = 0
    maze[2*height-1][2*width-1] = 0
    return maze

# Initialize maze logic and build physical wall occupancy grid
logic_maze = generate_maze_no_loops(M, N)
phys_h, phys_w = N * CELL_SIZE + WALL_THICK, M * CELL_SIZE + WALL_THICK
phys_maze = np.zeros((phys_h, phys_w))

for r in range(2 * N + 1):
    for c in range(2 * M + 1):
        if logic_maze[r][c] == 1:
            y_start = (r // 2) * CELL_SIZE + (WALL_THICK if r % 2 != 0 else 0)
            y_end = y_start + (PATH_WIDTH if r % 2 != 0 else WALL_THICK)
            x_start = (c // 2) * CELL_SIZE + (WALL_THICK if c % 2 != 0 else 0)
            x_end = x_start + (PATH_WIDTH if c % 2 != 0 else WALL_THICK)
            phys_maze[y_start:y_end, x_start:x_end] = 1

# Setup control telemetry parameters
cmd_vx, cmd_vy, cmd_w = 0.0, 0.0, 0.0
last_cmd_time = time.time()
robot_armed = False

# Compute start coordinates and endpoint (goal) target markers
start_x, start_y = float(WALL_THICK + PATH_WIDTH / 2), float(WALL_THICK + PATH_WIDTH / 2)
goal_x, goal_y = float((M-1) * CELL_SIZE + WALL_THICK + PATH_WIDTH / 2), float((N-1) * CELL_SIZE + WALL_THICK + PATH_WIDTH / 2)
robot_x, robot_y, robot_heading = start_x, start_y, 0.0
ROBOT_SPEED_SCALE = 3.0 

def is_collision(x, y):
    """Validates the rectangular footprint corners of the drone against wall boundaries."""
    half_w = ROBOT_W_WIDTH / 2.2
    half_l = ROBOT_L_LENGTH / 2.2
    
    corners = [
        (x - half_w, y - half_l),
        (x + half_w, y - half_l),
        (x - half_w, y + half_l),
        (x + half_w, y + half_l)
    ]
    
    for cx, cy in corners:
        if not (0 <= int(cy) < phys_h and 0 <= int(cx) < phys_w) or phys_maze[int(cy), int(cx)] == 1:
            return True
    return False

# ============================================================
# WEBSOCKET SERVER SECURITY & COMMUNICATIONS
# ============================================================
KEY_HEX = os.getenv("ROBOT_HMAC_KEY")
clients, server_loop = set(), None

async def handle_client(websocket):
    global cmd_vx, cmd_vy, cmd_w, last_cmd_time, robot_armed
    try:
        # Secure HMAC-SHA256 handshake verification
        nonce = hex(random.getrandbits(128))[2:].zfill(32)
        await websocket.send(json.dumps({"nonce": nonce}))
        auth_data = json.loads(await asyncio.wait_for(websocket.recv(), 5.0))
        if not hmac.compare_digest(auth_data.get("auth", ""), hmac.new(bytes.fromhex(KEY_HEX), bytes.fromhex(nonce), hashlib.sha256).hexdigest()):
            return
        await websocket.send(json.dumps({"auth": "ok"}))
        clients.add(websocket)
        
        async for msg in websocket:
            try:
                data = json.loads(msg)

                # Process system reset request from the HMI dashboard
                if data.get("cmd") == "reset":
                    reset_entire_simulation()
                    continue

                cmd_vx, cmd_vy, cmd_w = data.get("vx", 0.0), data.get("vy", 0.0), data.get("w", 0.0)
                robot_armed = True
                last_cmd_time = time.time()
            except: pass
    except: pass
    finally: clients.discard(websocket)

def start_ws():
    global server_loop
    server_loop = asyncio.new_event_loop()
    asyncio.set_event_loop(server_loop)
    async def run_server():
        async with websockets.serve(handle_client, "0.0.0.0", 8765):
            print("[API] WebSocket core server is active on port 8765...")
            await asyncio.Future() 
    server_loop.run_until_complete(run_server())

threading.Thread(target=start_ws, daemon=True).start()

def reset_entire_simulation():
    """Wipes current track, builds a new randomized maze topology and forces an immediate telemetry broadcast."""
    global logic_maze, phys_maze, robot_x, robot_y, robot_heading, robot_armed
    from config import M, N, CELL_SIZE, WALL_THICK, PATH_WIDTH, NUM_SENSORS, RAYS_PER_SENSOR, SENSOR_RANGE
    
    # 1. Generate new logical matrix
    logic_maze = generate_maze_no_loops(M, N)
    
    # 2. Compute new physical boundaries and clean the grid
    new_phys_h = N * CELL_SIZE + WALL_THICK
    new_phys_w = M * CELL_SIZE + WALL_THICK
    phys_maze = np.zeros((new_phys_h, new_phys_w))
    
    # 3. Build physical walls
    for r in range(2 * N + 1):
        for c in range(2 * M + 1):
            if logic_maze[r][c] == 1:
                y_start = (r // 2) * CELL_SIZE + (WALL_THICK if r % 2 != 0 else 0)
                y_end = y_start + (PATH_WIDTH if r % 2 != 0 else WALL_THICK)
                x_start = (c // 2) * CELL_SIZE + (WALL_THICK if c % 2 != 0 else 0)
                x_end = x_start + (PATH_WIDTH if c % 2 != 0 else WALL_THICK)
                phys_maze[y_start:y_end, x_start:x_end] = 1
                
    # 4. Teleport drone back to starting location and cut motors
    robot_x, robot_y = start_x, start_y
    robot_heading = 0.0
    robot_armed = False
    print("♻️ Server generated a new track and reset drone coordinates successfully!")

    # 5. Force an immediate network sync frame to all connected dashboards
    if clients and server_loop:
        at_meta = False
        total_rays = int(NUM_SENSORS * RAYS_PER_SENSOR)
        dummy_laser = [{"d": float(SENSOR_RANGE), "hit": False} for _ in range(total_rays)]
        
        msg = json.dumps({
            "pos_x": robot_x, "pos_y": robot_y, "heading": robot_heading, 
            "laser": dummy_laser, "at_meta": at_meta, "phys_maze": phys_maze.tolist()
        })
        for ws in list(clients):
            asyncio.run_coroutine_threadsafe(ws.send(msg), server_loop)
            
# ============================================================
# SIMULATOR KINEMATICS ENGINE LOOP
# ============================================================
async def physics_loop():
    global robot_x, robot_y, robot_heading, robot_armed
    while True:
        # Watchdog: failsafe timeout drops arming if network heartbeat flatlines
        if time.time() - last_cmd_time > 0.5:
            robot_armed = False

        if robot_armed:
            robot_heading += cmd_w * 0.1
            g_vx = cmd_vx * math.cos(robot_heading) - cmd_vy * math.sin(robot_heading)
            g_vy = cmd_vx * math.sin(robot_heading) + cmd_vy * math.cos(robot_heading)
            if not is_collision(robot_x + g_vx * ROBOT_SPEED_SCALE, robot_y): robot_x += g_vx * ROBOT_SPEED_SCALE
            if not is_collision(robot_x, robot_y + g_vy * ROBOT_SPEED_SCALE): robot_y += g_vy * ROBOT_SPEED_SCALE

        # Calculate laser sensor ray vectors via iterative wall boundaries checking
        all_hits = []
        for i in range(NUM_SENSORS):
            s_hdg = math.radians(math.degrees(robot_heading) + SENSOR_ANGLES_DEG[i])
            sx = robot_x + SENSOR_RADIUS * math.cos(s_hdg)
            sy = robot_y + SENSOR_RADIUS * math.sin(s_hdg)
            
            for ang in ray_angles_deg:
                r_ang = s_hdg + math.radians(ang)
                dx, dy = math.cos(r_ang), math.sin(r_ang)
                dist, hit = SENSOR_RANGE + random.gauss(0, NOISE_STD), False
                
                for d in range(1, int(dist)):
                    cx, cy = int(sx + dx*d), int(sy + dy*d)
                    if 0 <= cy < phys_h and 0 <= cx < phys_w:
                        if phys_maze[cy, cx] == 1:
                            dist, hit = d + random.gauss(0, NOISE_STD), True
                            break
                    else: break
                
                all_hits.append({"d": dist, "hit": hit})

        # Broadcast telemetry payload out to open clients
        if clients and server_loop:
            at_meta = bool(math.hypot(robot_x - goal_x, robot_y - goal_y) < 15.0)
            msg = json.dumps({
                "pos_x": robot_x, "pos_y": robot_y, "heading": robot_heading, 
                "laser": all_hits, "at_meta": at_meta, "phys_maze": phys_maze.tolist()
            })
            for ws in list(clients):
                asyncio.run_coroutine_threadsafe(ws.send(msg), server_loop)
                
        await asyncio.sleep(0.06)

def start_physics():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(physics_loop())

threading.Thread(target=start_physics, daemon=True).start()

# Keep script process alive
while True:
    time.sleep(1)