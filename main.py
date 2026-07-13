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

# Import central hardware and environment settings
from config import (
    N, M, CELL_SIZE, WALL_THICK, PATH_WIDTH,
    ROBOT_W_WIDTH, ROBOT_L_LENGTH, SENSOR_RADIUS, 
    SENSOR_RANGE, NOISE_STD, NUM_SENSORS, 
    RAYS_PER_SENSOR, SENSOR_ANGLES_DEG, ray_angles_deg,
    # Pre-computed radians for fast ray marching
    SENSOR_ANGLES_RAD, ray_angles_rad
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

# Thread lock for all shared state between physics loop and WebSocket handler
state_lock = threading.Lock()

# Setup control telemetry parameters
cmd_vx, cmd_vy, cmd_w = 0.0, 0.0, 0.0
last_cmd_time = time.time()
robot_armed = False
current_maze_id = 1  # Wersjonowanie sesji labiryntu

# Flag to pause physics during reset
physics_paused = False

# --- DZIEŃ 1: DYNAMICZNE USTAWIANIE METS I PRĘDKOŚCI SYMULACJI ---
SIM_SPEED = 1.0
META_PLACEMENT = "corner" # "corner" lub "center"

start_x, start_y = float(WALL_THICK + PATH_WIDTH / 2), float(WALL_THICK + PATH_WIDTH / 2)
goal_x, goal_y = 0.0, 0.0
goal_logic = (0, 0)

def update_goal_coordinates():
    global goal_x, goal_y, goal_logic
    if META_PLACEMENT == "corner":
        goal_logic = (2 * (N - 1) + 1, 2 * (M - 1) + 1)
        goal_x = float((M-1) * CELL_SIZE + WALL_THICK + PATH_WIDTH / 2)
        goal_y = float((N-1) * CELL_SIZE + WALL_THICK + PATH_WIDTH / 2)
    else: # "center"
        goal_logic = (2 * (N // 2) + 1, 2 * (M // 2) + 1)
        goal_x = float((M // 2) * CELL_SIZE + WALL_THICK + PATH_WIDTH / 2)
        goal_y = float((N // 2) * CELL_SIZE + WALL_THICK + PATH_WIDTH / 2)

update_goal_coordinates()
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
clients = set()
server_loop = None

def _broadcast_done(ws, fut):
    """Callback for async broadcast — cleans up on error without blocking."""
    try:
        fut.result()
    except (ConnectionResetError, websockets.exceptions.ConnectionClosed):
        clients.discard(ws)
    except Exception as e:
        log.warning(f"Broadcast send error: {e}")
        clients.discard(ws)

def broadcast_message(msg_dict):
    """Thread-safe non-blocking broadcast to all connected WebSocket clients."""
    global server_loop
    if not clients or not server_loop:
        return
    msg = json.dumps(msg_dict)
    loop = server_loop
    for ws in list(clients):
        try:
            fut = asyncio.run_coroutine_threadsafe(ws.send(msg), loop)
            fut.add_done_callback(lambda f, ws=ws: _broadcast_done(ws, f))
        except Exception as e:
            log.warning(f"Broadcast error: {e}")
            clients.discard(ws)

async def handle_client(websocket):
    global cmd_vx, cmd_vy, cmd_w, last_cmd_time, robot_armed, robot_x, robot_y, robot_heading, current_maze_id, SIM_SPEED, META_PLACEMENT
    try:
        # Secure HMAC-SHA256 handshake verification
        nonce = hex(random.getrandbits(128))[2:].zfill(32)
        await websocket.send(json.dumps({"nonce": nonce}))
        auth_data = json.loads(await asyncio.wait_for(websocket.recv(), 5.0))
        if not hmac.compare_digest(auth_data.get("auth", ""), hmac.new(bytes.fromhex(KEY_HEX), bytes.fromhex(nonce), hashlib.sha256).hexdigest()):
            log.warning("Auth failed — closing connection")
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
        log.info(f"Client connected ({len(clients)} total)")
        
        async for msg in websocket:
            try:
                data = json.loads(msg)

                if data.get("cmd") == "reset":
                    log.info("Received reset command")
                    reset_entire_simulation()
                    continue

                if data.get("cmd") == "teleport_to_start":
                    with state_lock:
                        cmd_vx, cmd_vy, cmd_w = 0.0, 0.0, 0.0
                        robot_x, robot_y = start_x, start_y
                        robot_heading = 0.0
                    log.info("Teleportacja drona na blok startowy wykonana!")
                    continue

                # --- OBSŁUGA DYNAMICZNYCH USTAWIEŃ Z DASHBOARDU ---
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
            except json.JSONDecodeError:
                log.warning("Invalid JSON from client")
            except Exception as e:
                log.error(f"Error processing message: {e}")
    except asyncio.TimeoutError:
        log.warning("Client auth timeout")
    except websockets.exceptions.ConnectionClosed:
        log.info("Client disconnected normally")
    except Exception as e:
        log.error(f"Client handler error: {e}")
    finally:
        clients.discard(websocket)
        log.info(f"Client removed ({len(clients)} remaining)")

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
    global logic_maze, phys_maze, phys_h, phys_w, robot_x, robot_y, robot_heading, robot_armed, current_maze_id
    from config import M, N, CELL_SIZE, WALL_THICK, PATH_WIDTH, NUM_SENSORS, RAYS_PER_SENSOR, SENSOR_RANGE
    
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
    
    log.info(f"♻️ Server generated a new track (ID: {current_maze_id}) with meta at {META_PLACEMENT}!")

    # Broadcast reset notification to all clients
    broadcast_message({
        "pos_x": robot_x, "pos_y": robot_y, "heading": robot_heading, 
        "laser": [{"d": float(SENSOR_RANGE), "hit": False} for _ in range(int(NUM_SENSORS * RAYS_PER_SENSOR))],
        "at_meta": bool(math.hypot(robot_x - goal_x, robot_y - goal_y) < 15.0),
        "maze_id": current_maze_id, "goal_cell": goal_logic,
        "phys_maze": phys_maze.tolist()
    })
            
# ============================================================
# SIMULATOR KINEMATICS ENGINE LOOP
# ============================================================
async def physics_loop():
    global robot_x, robot_y, robot_heading, robot_armed
    idle_heartbeat = 0.0
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

        now = time.time()
        robot_moved = False

        if not local_paused:
            if now - local_last_cmd > 0.5:
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
                        robot_moved = True
                    if not is_collision(robot_x, robot_y + g_vy * ROBOT_SPEED_SCALE):
                        robot_y += g_vy * ROBOT_SPEED_SCALE
                        robot_moved = True
                    if abs(local_w) > 0.001:
                        robot_heading = new_heading
                        robot_moved = True

            # --- ADAPTIVE BROADCAST ---
            # Snapshot shared state for broadcast under lock
            with state_lock:
                snap_x = robot_x
                snap_y = robot_y
                snap_heading = robot_heading
                snap_maze = phys_maze
                snap_h = phys_h
                snap_w = phys_w
                snap_maze_id = current_maze_id
                snap_goal_x = goal_x
                snap_goal_y = goal_y
                snap_goal_logic = goal_logic

            if robot_moved:
                # Full broadcast with laser data (robot is moving)
                all_hits = []
                h_cos = math.cos(snap_heading)
                h_sin = math.sin(snap_heading)
                
                for i in range(NUM_SENSORS):
                    # Sensor mounting in global frame
                    s_ang = snap_heading + SENSOR_ANGLES_RAD[i]
                    s_cos = math.cos(s_ang)
                    s_sin = math.sin(s_ang)
                    sx = snap_x + SENSOR_RADIUS * s_cos
                    sy = snap_y + SENSOR_RADIUS * s_sin
                    
                    for ang_rad in ray_angles_rad:
                        r_ang = s_ang + ang_rad
                        dx = math.cos(r_ang)
                        dy = math.sin(r_ang)
                        
                        # Single noise sample per ray
                        noise = random.gauss(0, NOISE_STD)
                        max_dist = float(SENSOR_RANGE + noise)
                        hit_dist = max_dist
                        hit = False
                        
                        # Step by 2 — pixel-perfect not needed for 10-wide robot
                        max_steps = min(int(max_dist), SENSOR_RANGE)
                        for d in range(1, max_steps, 2):
                            cx = int(sx + dx * d)
                            cy = int(sy + dy * d)
                            if 0 <= cy < snap_h and 0 <= cx < snap_w:
                                if snap_maze[cy, cx] == 1:
                                    hit_dist = float(d + noise)
                                    hit = True
                                    break
                            else:
                                break
                        
                        all_hits.append({"d": max(1.0, hit_dist), "hit": hit})

                at_meta = bool(math.hypot(snap_x - snap_goal_x, snap_y - snap_goal_y) < 15.0)
                broadcast_message({
                    "pos_x": snap_x, "pos_y": snap_y, "heading": snap_heading,
                    "laser": all_hits, "at_meta": at_meta,
                    "maze_id": snap_maze_id, "goal_cell": snap_goal_logic
                })
            elif now - idle_heartbeat > 1.0:
                # Idle heartbeat — just position, no laser data (saves CPU + bandwidth)
                idle_heartbeat = now
                at_meta = bool(math.hypot(snap_x - snap_goal_x, snap_y - snap_goal_y) < 15.0)
                broadcast_message({
                    "pos_x": snap_x, "pos_y": snap_y, "heading": snap_heading,
                    "at_meta": at_meta, "maze_id": snap_maze_id, "goal_cell": snap_goal_logic
                })

        # Adaptive sleep: tight when armed, relaxed when idle
        sleep_s = 0.03 / max(0.1, local_sim_speed) if local_armed else 0.1
        await asyncio.sleep(sleep_s)

def start_physics():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(physics_loop())

threading.Thread(target=start_physics, daemon=True).start()

while True:
    time.sleep(1)