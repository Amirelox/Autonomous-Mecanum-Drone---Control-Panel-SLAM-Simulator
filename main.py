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
import logging
from concurrent.futures import TimeoutError
from dotenv import load_dotenv

logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(threadName)s] %(levelname)s: %(message)s')
log = logging.getLogger(__name__)

load_dotenv()

from config import (
    N, M, CELL_SIZE, WALL_THICK, PATH_WIDTH,
    ROBOT_W_WIDTH, ROBOT_L_LENGTH, SENSOR_RADIUS, 
    SENSOR_RANGE, NOISE_STD, NUM_SENSORS, 
    RAYS_PER_SENSOR, SENSOR_ANGLES_DEG, ray_angles_deg,
    SENSOR_ANGLES_RAD, ray_angles_rad,
    ROBOT_SPEED_SCALE
)

def generate_maze_no_loops(width, height):
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

state_lock = threading.Lock()
cmd_vx, cmd_vy, cmd_w = 0.0, 0.0, 0.0
last_cmd_time = time.time()
robot_armed = False
current_maze_id = 1
physics_paused = False

SIM_SPEED = 1.0
META_PLACEMENT = "corner"

start_x, start_y = float(WALL_THICK + PATH_WIDTH / 2), float(WALL_THICK + PATH_WIDTH / 2)
goal_x, goal_y = 0.0, 0.0
goal_logic = (0, 0)



def update_goal_coordinates():
    global goal_x, goal_y, goal_logic
    if META_PLACEMENT == "corner":
        goal_logic = (2 * (N - 1) + 1, 2 * (M - 1) + 1)
        goal_x = float((M-1) * CELL_SIZE + WALL_THICK + PATH_WIDTH / 2)
        goal_y = float((N-1) * CELL_SIZE + WALL_THICK + PATH_WIDTH / 2)
    else:
        goal_logic = (2 * (N // 2) + 1, 2 * (M // 2) + 1)
        goal_x = float((M // 2) * CELL_SIZE + WALL_THICK + PATH_WIDTH / 2)
        goal_y = float((N // 2) * CELL_SIZE + WALL_THICK + PATH_WIDTH / 2)

update_goal_coordinates()
robot_x, robot_y, robot_heading = start_x, start_y, 0.0

def is_collision(x, y):
    half_w = ROBOT_W_WIDTH / 2.3
    half_l = ROBOT_L_LENGTH / 2.3
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

KEY_HEX = os.getenv("ROBOT_HMAC_KEY")
clients = set()
server_loop = None

def _broadcast_done(ws, fut):
    try:
        fut.result()
    except Exception:
        clients.discard(ws)

def broadcast_message(msg_dict):
    global server_loop
    if not clients or not server_loop:
        return
    msg = json.dumps(msg_dict)
    for ws in list(clients):
        try:
            fut = asyncio.run_coroutine_threadsafe(ws.send(msg), server_loop)
            fut.add_done_callback(lambda f, ws=ws: _broadcast_done(ws, f))
        except Exception:
            clients.discard(ws)

async def handle_client(websocket):
    global cmd_vx, cmd_vy, cmd_w, last_cmd_time, robot_armed, robot_x, robot_y, robot_heading, current_maze_id, SIM_SPEED, META_PLACEMENT
    try:
        nonce = hex(random.getrandbits(128))[2:].zfill(32)
        await websocket.send(json.dumps({"nonce": nonce}))
        auth_data = json.loads(await asyncio.wait_for(websocket.recv(), 5.0))
        if not hmac.compare_digest(auth_data.get("auth", ""), hmac.new(bytes.fromhex(KEY_HEX), bytes.fromhex(nonce), hashlib.sha256).hexdigest()):
            return
        await websocket.send(json.dumps({"auth": "ok"}))
        clients.add(websocket)
        
        with state_lock:
            initial_maze = phys_maze.tolist()
            initial_maze_id = current_maze_id
            initial_goal = goal_logic
        
        await websocket.send(json.dumps({
            "phys_maze": initial_maze,
            "maze_id": initial_maze_id,
            "goal_cell": initial_goal
        }))
        
        async for msg in websocket:
            try:
                data = json.loads(msg)
                if data.get("cmd") == "reset":
                    reset_entire_simulation()
                    continue
                if data.get("cmd") == "teleport_to_start":
                    with state_lock:
                        cmd_vx, cmd_vy, cmd_w = 0.0, 0.0, 0.0
                        robot_x, robot_y = start_x, start_y
                        robot_heading = 0.0
                    continue
                if data.get("cmd") == "set_speed":
                    with state_lock:
                        SIM_SPEED = float(data.get("value", 1.0))
                    continue
                if data.get("cmd") == "set_meta":
                    with state_lock:
                        META_PLACEMENT = data.get("value", "corner")
                        update_goal_coordinates()
                    continue

                with state_lock:
                    cmd_vx, cmd_vy, cmd_w = data.get("vx", 0.0), data.get("vy", 0.0), data.get("w", 0.0)
                    robot_armed = True
                    last_cmd_time = time.time()
            except Exception:
                pass
    except Exception:
        pass
    finally:
        clients.discard(websocket)

def start_ws():
    global server_loop
    server_loop = asyncio.new_event_loop()
    asyncio.set_event_loop(server_loop)
    async def run_server():
        async with websockets.serve(handle_client, "0.0.0.0", 8765):
            await asyncio.Future() 
    server_loop.run_until_complete(run_server())

threading.Thread(target=start_ws, daemon=True).start()

def reset_entire_simulation():
    global logic_maze, phys_maze, phys_h, phys_w, robot_x, robot_y, robot_heading, robot_armed, current_maze_id
    
    with state_lock:
        global physics_paused
        physics_paused = True
        current_maze_id += 1  
        logic_maze = generate_maze_no_loops(M, N)
        update_goal_coordinates()
        phys_h = N * CELL_SIZE + WALL_THICK
        phys_w = M * CELL_SIZE + WALL_THICK
        phys_maze = np.zeros((phys_h, phys_w))
        
        for r in range(2 * N + 1):
            for c in range(2 * M + 1):
                if logic_maze[r][c] == 1:
                    y_start = (r // 2) * CELL_SIZE + (WALL_THICK if r % 2 != 0 else 0)
                    y_end = y_start + (PATH_WIDTH if r % 2 != 0 else WALL_THICK)
                    x_start = (c // 2) * CELL_SIZE + (WALL_THICK if c % 2 != 0 else 0)
                    x_end = x_start + (PATH_WIDTH if c % 2 != 0 else WALL_THICK)
                    phys_maze[y_start:y_end, x_start:x_end] = 1
                    
        robot_x, robot_y = start_x, start_y
        robot_heading = 0.0
        robot_armed = False
        physics_paused = False

async def physics_loop():
    global robot_x, robot_y, robot_heading, robot_armed
    
    while True:
        with state_lock:
            local_paused = physics_paused
            local_armed = robot_armed
            local_heading = robot_heading
            local_vx = cmd_vx
            local_vy = cmd_vy
            local_w = cmd_w
            local_last_cmd = last_cmd_time
            local_sim_speed = SIM_SPEED

        if local_paused:
            await asyncio.sleep(0.05)
            continue

        if time.time() - local_last_cmd > 0.5:
            with state_lock:
                robot_armed = False
            local_armed = False

        if local_armed:
            new_heading = local_heading + local_w * 0.1
            
            g_vx = local_vx * math.cos(new_heading) - local_vy * math.sin(new_heading)
            g_vy = local_vx * math.sin(new_heading) + local_vy * math.cos(new_heading)
            
            with state_lock:
                if not is_collision(robot_x + g_vx * ROBOT_SPEED_SCALE, robot_y):
                    robot_x += g_vx * ROBOT_SPEED_SCALE
                if not is_collision(robot_x, robot_y + g_vy * ROBOT_SPEED_SCALE):
                    robot_y += g_vy * ROBOT_SPEED_SCALE
                if abs(local_w) > 0.001:
                    robot_heading = new_heading

        with state_lock:
            snap_x = robot_x
            snap_y = robot_y
            snap_heading = robot_heading
            snap_maze_id = current_maze_id
            snap_goal_logic = goal_logic

        all_hits = []
        for i in range(NUM_SENSORS):
            s_ang = snap_heading + SENSOR_ANGLES_RAD[i]
            sx = snap_x + SENSOR_RADIUS * math.cos(s_ang)
            sy = snap_y + SENSOR_RADIUS * math.sin(s_ang)
            
            for ang_rad in ray_angles_rad:
                r_ang = s_ang + ang_rad
                dx = math.cos(r_ang)
                dy = math.sin(r_ang)
                
                noise = random.gauss(0, NOISE_STD)
                max_dist = float(SENSOR_RANGE + noise)
                hit_dist = max_dist
                hit = False
                
                max_steps = min(int(max_dist), SENSOR_RANGE)
                for d in range(1, max_steps, 2):
                    cx = int(sx + dx * d)
                    cy = int(sy + dy * d)
                    if 0 <= cy < phys_h and 0 <= cx < phys_w:
                        if phys_maze[cy, cx] == 1:
                            hit_dist = float(d + noise)
                            hit = True
                            break
                    else:
                        break
                all_hits.append({"d": max(1.0, hit_dist), "hit": hit})

        at_meta = bool(math.hypot(snap_x - goal_x, snap_y - goal_y) < 15.0)
        
        broadcast_message({
            "pos_x": snap_x, "pos_y": snap_y, "heading": snap_heading,
            "laser": all_hits, "at_meta": at_meta,
            "maze_id": snap_maze_id, "goal_cell": snap_goal_logic
        })

        await asyncio.sleep(0.02 / max(0.1, local_sim_speed))

def start_physics():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(physics_loop())

threading.Thread(target=start_physics, daemon=True).start()

while True:
    time.sleep(1)