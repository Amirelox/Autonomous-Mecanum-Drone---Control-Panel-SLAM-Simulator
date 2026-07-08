import asyncio
import json
import hmac
import hashlib
import math
import numpy as np
import websockets
import threading
import queue
import os
from dotenv import load_dotenv

load_dotenv()

# Import centralized configuration constants
from config import (
    CELL_SIZE, WALL_THICK, PATH_WIDTH,
    ROBOT_W_WIDTH, ROBOT_L_LENGTH, SENSOR_RADIUS, 
    SENSOR_RANGE, NUM_SENSORS, RAYS_PER_SENSOR, 
    SENSOR_ANGLES_DEG, ray_angles_deg
)

# ============================================================
# SLAM INTERNAL RESOLUTION MAP CONFIG
# ============================================================
LOGIC_ROWS, LOGIC_COLS = 41, 41
PHYS_H, PHYS_W = 500, 500

KEY_HEX = os.getenv("ROBOT_HMAC_KEY")
ROBOT_WS = "ws://localhost:8765"

# Probabilistic grid array initializing everything to unknown territory (0.5)
prob_map = np.full((PHYS_H, PHYS_W), 0.5)

def logic_to_phys(log_r, log_c):
    return (log_c // 2) * CELL_SIZE + WALL_THICK + PATH_WIDTH / 2, (log_r // 2) * CELL_SIZE + WALL_THICK + PATH_WIDTH / 2

def phys_to_logic(phys_x, phys_y):
    return int((phys_y // CELL_SIZE) * 2 if (phys_y % CELL_SIZE) < WALL_THICK else (phys_y // CELL_SIZE) * 2 + 1), \
           int((phys_x // CELL_SIZE) * 2 if (phys_x % CELL_SIZE) < WALL_THICK else (phys_x // CELL_SIZE) * 2 + 1)

def snap_hit_to_logic(phys_x, phys_y):
    """Snaps a point to the nearest discrete wall coordinate on the logical grid maps."""
    x_idx, y_idx = round(phys_x / CELL_SIZE), round(phys_y / CELL_SIZE)
    dist_x, dist_y = abs(phys_x - x_idx * CELL_SIZE), abs(phys_y - y_idx * CELL_SIZE)
    if dist_x > 6.0 and dist_y > 6.0:
        return -1, -1
    if dist_x < 6.0 and dist_y < 6.0:
        return y_idx * 2, x_idx * 2
    return (int(phys_y // CELL_SIZE) * 2 + 1, x_idx * 2) if dist_x < dist_y else (y_idx * 2, int(phys_x // CELL_SIZE) * 2 + 1)

class DFSController:
    def __init__(self):
        # Safety feature: initializes in finished/paused mode to prevent immediate autostart
        self.finished = True 
        self.logic_map = [[-1] * LOGIC_COLS for _ in range(LOGIC_ROWS)]
        self.logic_map[1][1] = 0
        self.start = (1, 1)
        self.current_logic_pos = self.start
        self.visited = {self.start}
        self.path_stack = []
        self.target_logic = None
        self.target_phys = None
        self.heading, self.pos_x, self.pos_y = 0.0, float(logic_to_phys(1,1)[0]), float(logic_to_phys(1,1)[1])
        self.laser_data = []
        self.last_vx, self.last_vy = 0.0, 0.0
        self.stuck_frames = 0  
        self.min_seen_x, self.max_seen_x = self.pos_x, self.pos_x
        self.min_seen_y, self.max_seen_y = self.pos_y, self.pos_y
        self.phys_maze_data = None

    def update_map(self, pos_x, pos_y, heading, laser_data):
        """Processes ray hits to clear paths or plot walls onto the occupancy array maps."""
        expand_map_if_needed(pos_x, pos_y, self)
        self.pos_x, self.pos_y, self.heading = pos_x, pos_y, heading
        self.laser_data = laser_data

        self.min_seen_x = min(self.min_seen_x, pos_x)
        self.max_seen_x = max(self.max_seen_x, pos_x)
        self.min_seen_y = min(self.min_seen_y, pos_y)
        self.max_seen_y = max(self.max_seen_y, pos_y)

        for i in range(NUM_SENSORS):
            s_hdg = math.radians(math.degrees(heading) + SENSOR_ANGLES_DEG[i])
            sx = pos_x + SENSOR_RADIUS * math.cos(s_hdg)
            sy = pos_y + SENSOR_RADIUS * math.sin(s_hdg)
            for j in range(RAYS_PER_SENSOR):
                r_ang = s_hdg + math.radians(ray_angles_deg[j])
                dx, dy = math.cos(r_ang), math.sin(r_ang)
                dist = laser_data[i * RAYS_PER_SENSOR + j]["d"]
                is_hit = laser_data[i * RAYS_PER_SENSOR + j]["hit"]

                c_range = min(dist, SENSOR_RANGE)
                # Cast free space probability updates
                for d in range(1, int(c_range), 2):
                    cy, cx = int(sy + dy*d), int(sx + dx*d)
                    if 0 <= cy < PHYS_H and 0 <= cx < PHYS_W:
                        prob_map[cy, cx] = max(0.0, prob_map[cy, cx] - 0.05)

                if c_range >= 1:
                    for d in np.arange(0, max(0.0, c_range - 12.0), 2.0):
                        r, c = phys_to_logic(sx + dx*d, sy + dy*d)
                        if 0 <= r < LOGIC_ROWS and 0 <= c < LOGIC_COLS and self.logic_map[r][c] == -1:
                            self.logic_map[r][c] = 0

                # Cast wall obstacle occupancy updates
                if is_hit:
                    hy, hx = int(sy + dy*dist), int(sx + dx*dist)
                    if 0 <= hy < PHYS_H and 0 <= hx < PHYS_W:
                        y_min, y_max = max(0, hy-1), min(PHYS_H, hy+2)
                        x_min, x_max = max(0, hx-1), min(PHYS_W, hx+2)
                        prob_map[y_min:y_max, x_min:x_max] = min(1.0, prob_map[hy, hx] + 0.3)

                        wr, wc = snap_hit_to_logic(sx + dx*dist, sy + dy*dist)
                        if 0 <= wr < LOGIC_ROWS and 0 <= wc < LOGIC_COLS:
                            if prob_map[hy, hx] > 0.8:
                                if wr % 2 == 0 and wc % 2 == 0:
                                    pass
                                elif (wr, wc) == self.current_logic_pos or (wr, wc) == self.target_logic:
                                    pass
                                else:
                                    self.logic_map[wr][wc] = 1

    def find_next_target(self):
        """Scans adjacent nodes to select the next step for DFS exploration."""
        r, c = self.current_logic_pos
        for dr, dc in [(0, 2), (2, 0), (0, -2), (-2, 0)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < LOGIC_ROWS and 0 <= nc < LOGIC_COLS:
                mid_free = (self.logic_map[r + dr//2][c + dc//2] <= 0)
                target_free = (self.logic_map[nr][nc] <= 0)
                if mid_free and target_free and (nr, nc) not in self.visited:
                    self.visited.add((nr, nc))
                    self.path_stack.append((r, c))
                    return (nr, nc)
        return self.path_stack.pop() if self.path_stack else None

    def update_target(self):
        if self.finished:
            self.target_logic = None
            self.target_phys = None
            return

        if self.target_logic is None:
            nxt = self.find_next_target()
            if nxt:
                self.target_logic = nxt
                self.target_phys = logic_to_phys(nxt[0], nxt[1])
                self.last_vx, self.last_vy = 0.0, 0.0

    def get_command(self):
        # --- CRITICAL FIX: IF THE SERVER NOTIFIES FINISHED/AT_META, STOP IMMEDIATELY ---
        if self.finished or self.target_phys is None:
            # Check if there are manual overrides from the dashboard console
            manual_vx = getattr(self, 'last_vx', 0.0)
            manual_vy = getattr(self, 'last_vy', 0.0)
            return {"vx": manual_vx, "vy": manual_vy, "w": 0.0}

        # --- DYNAMIC WALL OFFSET BRAKING LOGIC ---
        tx, ty = self.target_phys
        
        if self.target_logic == (LOGIC_ROWS - 2, LOGIC_COLS - 2):
            diff_x = tx - self.pos_x
            diff_y = ty - self.pos_y
            safety_offset = 24.0
            
            if abs(diff_x) > abs(diff_y):
                if diff_x > 0: tx -= safety_offset
                else: tx += safety_offset
            else:
                if diff_y > 0: ty -= safety_offset
                else: ty += safety_offset

        real_dx, real_dy = tx - self.pos_x, ty - self.pos_y
        move_speed = 0.60

        if abs(real_dx) > abs(real_dy):
            global_vx = move_speed if real_dx > 0 else -move_speed
            global_vy = 0.0
        else:
            global_vx = 0.0
            global_vy = move_speed if real_dy > 0 else -move_speed

        local_vx = global_vx * math.cos(self.heading) + global_vy * math.sin(self.heading)
        local_vy = -global_vx * math.sin(self.heading) + global_vy * math.cos(self.heading)

        self.last_vx = local_vx
        self.last_vy = local_vy
        
        return {"vx": local_vx, "vy": local_vy, "w": 0.0}

controller = DFSController()

def expand_map_if_needed(phys_x, phys_y, controller):
    """Pads the numpy map structures dynamically if drone voyages near borders."""
    global PHYS_H, PHYS_W, LOGIC_ROWS, LOGIC_COLS, prob_map
    margin = 250
    needs_expansion = False
    new_h, new_w = PHYS_H, PHYS_W

    if phys_y + margin > PHYS_H:
        new_h = int(phys_y + margin + 300)
        needs_expansion = True
    if phys_x + margin > PHYS_W:
        new_w = int(phys_x + margin + 300)
        needs_expansion = True

    if needs_expansion:
        pad_h, pad_w = max(0, new_h - PHYS_H), max(0, new_w - PHYS_W)
        prob_map = np.pad(prob_map, ((0, pad_h), (0, pad_w)), constant_values=0.5)
        PHYS_H, PHYS_W = new_h, new_w
        new_logic_rows = int((PHYS_H // CELL_SIZE) * 2 + 1)
        new_logic_cols = int((PHYS_W // CELL_SIZE) * 2 + 1)

        for r in range(len(controller.logic_map)):
            diff_cols = new_logic_cols - len(controller.logic_map[r])
            if diff_cols > 0:
                controller.logic_map[r].extend([-1] * diff_cols)

        while len(controller.logic_map) < new_logic_rows:
            controller.logic_map.append([-1] * new_logic_cols)
        LOGIC_ROWS, LOGIC_COLS = new_logic_rows, new_logic_cols

# Thread-safe pipeline queue for system interaction commands
command_queue = queue.Queue()

async def client_loop():
    try:
        async with websockets.connect(ROBOT_WS, ping_interval=None, ping_timeout=None) as ws:
            nonce = json.loads(await ws.recv())["nonce"]
            await ws.send(json.dumps({"auth": hmac.new(bytes.fromhex(KEY_HEX), bytes.fromhex(nonce), hashlib.sha256).hexdigest()}))
            if json.loads(await ws.recv()).get("auth") != "ok":
                return

            with open("api_client.log", "w", encoding="utf-8") as log_file:
                log_file.write("=== LOGS INITIALIZATION SUCCESSFUL ===\n")

            async for msg in ws:
                data = json.loads(msg)
                if "laser" not in data:
                    continue

                if "phys_maze" in data:
                    controller.phys_maze_data = data["phys_maze"]

                # --- STALL DETECTOR MECHANISM ----
                if controller.target_logic is not None and not controller.finished:
                    dist_moved = math.hypot(data["pos_x"] - controller.pos_x, data["pos_y"] - controller.pos_y)
                    if dist_moved < 0.05:
                        controller.stuck_frames += 1
                    else:
                        controller.stuck_frames = 0

                    if controller.stuck_frames >= 20:
                        r1, c1 = controller.current_logic_pos
                        r2, c2 = controller.target_logic
                        mid_r, mid_c = (r1 + r2) // 2, (c1 + c2) // 2
                        print(f"🛑 Stall Detected! Mapping wirtual wall obstacle at cell: ({mid_r}, {mid_c})")
                        controller.logic_map[mid_r][mid_c] = 1 
                        if controller.target_logic in controller.visited:
                            controller.visited.remove(controller.target_logic)
                        if controller.path_stack:
                            controller.path_stack.pop()
                        controller.target_logic = None
                        controller.target_phys = None
                        controller.stuck_frames = 0
                else:
                    controller.stuck_frames = 0

                controller.update_map(data["pos_x"], data["pos_y"], data["heading"], data["laser"])

                # Handle endpoint goal detection triggers
                if data.get("at_meta", False) and not controller.finished:
                    controller.finished = True
                    controller.target_logic = None
                    controller.target_phys = None
                    
                    controller.last_vx = 0.0
                    controller.last_vy = 0.0
                    
                    await ws.send(json.dumps({"vx": 0.0, "vy": 0.0, "w": 0.0}))
                    print("🏁 GOAL REACHED! EMERGENCY BRAKE APPLIED!")
                    continue

                if controller.target_phys is not None:
                    tx, ty = controller.target_phys
                    dist_to_target = math.hypot(tx - data["pos_x"], ty - data["pos_y"])
                    if dist_to_target < 5.0:
                        controller.current_logic_pos = controller.target_logic
                        controller.target_logic = None
                        controller.target_phys = None

                if controller.target_logic is None and not controller.finished:
                    nl = phys_to_logic(data["pos_x"], data["pos_y"])
                    controller.current_logic_pos = nl
                    controller.update_target()

                # --- SCAN ACTION COMMITS QUEUE ---
                try:
                    cmd_from_dashboard = command_queue.get_nowait()
                    if cmd_from_dashboard == "reset":
                        await ws.send(json.dumps({"cmd": "reset"}))
                        command_queue.task_done()
                        continue
                except queue.Empty:
                    pass

                command = controller.get_command()
                json_request = json.dumps(command)
                await ws.send(json_request)

                with open("api_client.log", "a", encoding="utf-8") as log_file:
                    log_file.write(
                        f"Pos: {controller.current_logic_pos} | "
                        f"Target: {controller.target_logic} | "
                        f"StuckFrames: {controller.stuck_frames} | "
                        f"REQ -> {json_request}\n"
                    )

    except Exception as e:
        print("Network Client error context:", e)

# Launch client processes thread safety checking
if not any(t.name == "ClientWSThread" for t in threading.enumerate()):
    threading.Thread(target=lambda: asyncio.run(client_loop()), name="ClientWSThread", daemon=True).start()