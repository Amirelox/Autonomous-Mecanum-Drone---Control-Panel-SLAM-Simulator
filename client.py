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
import time
import logging
from dotenv import load_dotenv

load_dotenv()

logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(name)s] %(levelname)s: %(message)s')
log = logging.getLogger("client")

from config import (
    N, M, CELL_SIZE, WALL_THICK, PATH_WIDTH,
    ROBOT_W_WIDTH, ROBOT_L_LENGTH, SENSOR_RADIUS, 
    SENSOR_RANGE, NUM_SENSORS, RAYS_PER_SENSOR, 
    SENSOR_ANGLES_DEG, ray_angles_deg,
    SENSOR_ANGLES_RAD, ray_angles_rad
)

LOGIC_ROWS, LOGIC_COLS = 41, 41
PHYS_H, PHYS_W = 500, 500
KEY_HEX = os.getenv("ROBOT_HMAC_KEY")
ROBOT_WS = "ws://localhost:8765"

prob_map = np.full((PHYS_H, PHYS_W), 0.5)

def logic_to_phys(log_r, log_c):
    return (log_c // 2) * CELL_SIZE + WALL_THICK + PATH_WIDTH / 2, (log_r // 2) * CELL_SIZE + WALL_THICK + PATH_WIDTH / 2

def phys_to_logic(phys_x, phys_y):
    return int((phys_y // CELL_SIZE) * 2 if (phys_y % CELL_SIZE) < WALL_THICK else (phys_y // CELL_SIZE) * 2 + 1), \
           int((phys_x // CELL_SIZE) * 2 if (phys_x % CELL_SIZE) < WALL_THICK else (phys_x // CELL_SIZE) * 2 + 1)

def snap_hit_to_logic(phys_x, phys_y):
    x_idx, y_idx = round(phys_x / CELL_SIZE), round(phys_y / CELL_SIZE)
    dist_x, dist_y = abs(phys_x - x_idx * CELL_SIZE), abs(phys_y - y_idx * CELL_SIZE)
    if dist_x > 6.0 and dist_y > 6.0:
        return -1, -1
    if dist_x < 6.0 and dist_y < 6.0:
        return y_idx * 2, x_idx * 2
    return (int(phys_y // CELL_SIZE) * 2 + 1, x_idx * 2) if dist_x < dist_y else (y_idx * 2, int(phys_x // CELL_SIZE) * 2 + 1)

class DFSController:
    def __init__(self):
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

        self.goal_cell = (2 * (N - 1) + 1, 2 * (M - 1) + 1)
        self.fast_run = False
        self.teleporting = False
        self.current_maze_id = 1
        self.optimized_path = []
        self.exploration_path = [self.start]

        self.run_start_time = None
        self.exploration_duration = 0.0
        self.fast_run_duration = 0.0
        self.fast_run_start_time = None

        self.benchmark_mode = False
        self.current_benchmark_run = 0
        self.fast_run_times = []

        self.max_logic_r = 1          
        self.max_logic_c = 1          
        self.meta_placement = "corner" 
        self.discovered_n = None      
        self.discovered_m = None      
        self.computed_goal = None     
        self.return_path = []         
        self.exploration_done = False 
        self.finalizing = False       
        self._lock = threading.Lock()

    def update_map(self, pos_x, pos_y, heading, laser_data):
        expand_map_if_needed(pos_x, pos_y, self)
        self.pos_x, self.pos_y, self.heading = pos_x, pos_y, heading
        self.laser_data = laser_data

        self.min_seen_x = min(self.min_seen_x, pos_x)
        self.max_seen_x = max(self.max_seen_x, pos_x)
        self.min_seen_y = min(self.min_seen_y, pos_y)
        self.max_seen_y = max(self.max_seen_y, pos_y)

        for i in range(NUM_SENSORS):
            s_hdg = heading + SENSOR_ANGLES_RAD[i]
            sx = pos_x + SENSOR_RADIUS * math.cos(s_hdg)
            sy = pos_y + SENSOR_RADIUS * math.sin(s_hdg)
            for j in range(RAYS_PER_SENSOR):
                r_ang = s_hdg + ray_angles_rad[j]
                dx, dy = math.cos(r_ang), math.sin(r_ang)
                dist = laser_data[i * RAYS_PER_SENSOR + j]["d"]
                is_hit = laser_data[i * RAYS_PER_SENSOR + j]["hit"]

                c_range = min(dist, SENSOR_RANGE)
                
                # OPTYMALIZACJA CPU: Zwiększony krok z 2 do 10 pikseli (likwiduje lagi!)
                for d in range(1, int(c_range), 10):
                    cy, cx = int(sy + dy*d), int(sx + dx*d)
                    if 0 <= cy < PHYS_H and 0 <= cx < PHYS_W:
                        prob_map[cy, cx] = max(0.0, prob_map[cy, cx] - 0.04)

                if c_range >= 1:
                    for d in range(0, int(max(0.0, c_range - 10.0)), 15):
                        r, c = phys_to_logic(sx + dx*d, sy + dy*d)
                        if 0 <= r < LOGIC_ROWS and 0 <= c < LOGIC_COLS:
                            if self.logic_map[r][c] == -1 or self.logic_map[r][c] == 1:
                                lx, ly = int(sx + dx*d), int(sy + dy*d)
                                if 0 <= ly < PHYS_H and 0 <= lx < PHYS_W and prob_map[ly, lx] < 0.4:
                                    self.logic_map[r][c] = 0

                if is_hit:
                    hy, hx = int(sy + dy*dist), int(sx + dx*dist)
                    if 0 <= hy < PHYS_H and 0 <= hx < PHYS_W:
                        y_min, y_max = max(0, hy-1), min(PHYS_H, hy+2)
                        x_min, x_max = max(0, hx-1), min(PHYS_W, hx+2)
                        prob_map[y_min:y_max, x_min:x_max] = min(1.0, prob_map[hy, hx] + 0.25)

                        wr, wc = snap_hit_to_logic(sx + dx*dist, sy + dy*dist)
                        if 0 <= wr < LOGIC_ROWS and 0 <= wc < LOGIC_COLS:
                            if prob_map[hy, hx] > 0.75:
                                if wr % 2 == 0 and wc % 2 == 0:
                                    pass
                                elif (wr, wc) == self.current_logic_pos or (wr, wc) == self.target_logic:
                                    pass
                                else:
                                    self.logic_map[wr][wc] = 1

    def find_next_target(self):
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

    def track_bounding_box(self, logic_pos):
        r, c = logic_pos
        if r > self.max_logic_r: self.max_logic_r = r
        if c > self.max_logic_c: self.max_logic_c = c

    def compute_shortest_path(self, start, goal):
        from collections import deque
        queue_bfs = deque([start])
        parent = {start: None}
        while queue_bfs:
            curr = queue_bfs.popleft()
            if curr == goal:
                path = []
                while curr is not None:
                    path.append(curr)
                    curr = parent[curr]
                return list(reversed(path))
            r, c = curr
            for dr, dc in [(0, 2), (2, 0), (0, -2), (-2, 0)]:
                nr, nc = r + dr, c + dc
                if 0 <= nr < LOGIC_ROWS and 0 <= nc < LOGIC_COLS:
                    mid_free = (self.logic_map[r + dr//2][c + dc//2] == 0)
                    target_free = (self.logic_map[nr][nc] == 0)
                    if mid_free and target_free and (nr, nc) not in parent:
                        parent[(nr, nc)] = curr
                        queue_bfs.append((nr, nc))
        return [start]

    def _finalize_exploration(self):
        if self.finalizing: return
        self.finalizing = True

        if self.run_start_time is not None:
            self.exploration_duration = time.time() - self.run_start_time

        self.discovered_n = (self.max_logic_r + 1) // 2
        self.discovered_m = (self.max_logic_c + 1) // 2
        log.info(f"🟡 Wykryte gabaryty świata: {self.discovered_n}x{self.discovered_m}")

        if self.meta_placement == "corner":
            goal_r, goal_c = self.max_logic_r, self.max_logic_c
        else:
            goal_r = 2 * (self.discovered_n // 2) + 1
            goal_c = 2 * (self.discovered_m // 2) + 1
            
        self.computed_goal = (goal_r, goal_c)
        self.goal_cell = (goal_r, goal_c)

        for idx in range(len(self.exploration_path)):
            r, c = self.exploration_path[idx]
            self.logic_map[r][c] = 0
            if idx > 0:
                pr, pc = self.exploration_path[idx - 1]
                self.logic_map[(r + pr) // 2][(c + pc) // 2] = 0

        self.logic_map[goal_r][goal_c] = 0
        
        full_path = self.compute_shortest_path(self.start, self.computed_goal)
        self.optimized_path = list(full_path[1:]) if len(full_path) > 1 else []
        
        self.fast_run = True
        self.teleporting = True
        self.target_logic = None
        self.target_phys = None
        self.last_vx, self.last_vy = 0.0, 0.0
        self.exploration_done = True
        self.finalizing = False
        log.info("🏁 EKSPLORACJA KOŃCOWA ZAKOŃCZONA - Trwa ładowanie trasy BFS dla Fast Run...")

    def update_target(self):
        if self.finished:
            self.target_logic = None
            self.target_phys = None
            return

        if self.fast_run:
            if self.target_logic is None:
                if self.optimized_path:
                    nxt = self.optimized_path.pop(0)
                    self.target_logic = nxt
                    self.target_phys = logic_to_phys(nxt[0], nxt[1])
                    self.last_vx, self.last_vy = 0.0, 0.0
                else:
                    self.finished = True
            return

        if self.target_logic is None:
            nxt = self.find_next_target()
            if nxt:
                self.target_logic = nxt
                self.target_phys = logic_to_phys(nxt[0], nxt[1])
                self.last_vx, self.last_vy = 0.0, 0.0
                return

            if self.current_logic_pos == self.start:
                self.exploration_done = True
                return

            if not self.return_path:
                for idx in range(len(self.exploration_path)):
                    r, c = self.exploration_path[idx]
                    self.logic_map[r][c] = 0
                    if idx > 0:
                        pr, pc = self.exploration_path[idx-1]
                        self.logic_map[(r+pr)//2][(c+pc)//2] = 0

                path_home = self.compute_shortest_path(self.current_logic_pos, self.start)
                if len(path_home) > 1:
                    self.return_path = list(path_home[1:])

            if self.return_path:
                nxt = self.return_path.pop(0)
                self.target_logic = nxt
                self.target_phys = logic_to_phys(nxt[0], nxt[1])
                self.last_vx, self.last_vy = 0.0, 0.0

    def get_command(self):
        """Returns motion command with integrated cross-track error stabilization."""
        if self.finished or self.target_phys is None:
            return {"vx": 0.0, "vy": 0.0, "w": 0.0}

        tx, ty = self.target_phys
        
        if self.fast_run and self.target_logic == self.goal_cell:
            diff_x = tx - self.pos_x
            diff_y = ty - self.pos_y
            safety_offset = 6.0
            if abs(diff_x) > abs(diff_y):
                tx += -safety_offset if diff_x > 0 else safety_offset
            else:
                ty += -safety_offset if diff_y > 0 else safety_offset

        real_dx, real_dy = tx - self.pos_x, ty - self.pos_y
        
        move_speed = 1.25

        # Aktywne centrowanie w osiach korytarza labiryntu
        if abs(real_dx) > abs(real_dy):
            global_vx = move_speed if real_dx > 0 else -move_speed
            global_vy = max(-0.3, min(0.3, real_dy * 0.12))  
        else:
            global_vx = max(-0.3, min(0.3, real_dx * 0.12))  
            global_vy = move_speed if real_dy > 0 else -move_speed

        local_vx = global_vx * math.cos(self.heading) + global_vy * math.sin(self.heading)
        local_vy = -global_vx * math.sin(self.heading) + global_vy * math.cos(self.heading)

        self.last_vx = local_vx
        self.last_vy = local_vy
        
        return {"vx": local_vx, "vy": local_vy, "w": 0.0}

    def reset_logic_and_maps(self, keep_benchmark=False):
        global PHYS_H, PHYS_W, LOGIC_ROWS, LOGIC_COLS, prob_map
        PHYS_H, PHYS_W = 500, 500
        LOGIC_ROWS, LOGIC_COLS = 41, 41
        prob_map = np.full((500, 500), 0.5)
        
        self.finished = False if keep_benchmark else True
        self.logic_map = [[-1] * LOGIC_COLS for _ in range(LOGIC_ROWS)]
        self.logic_map[1][1] = 0
        self.visited = {(1, 1)}
        self.path_stack = []
        self.target_logic = None
        self.target_phys = None
        self.stuck_frames = 0
        self.fast_run = False
        self.teleporting = False
        self.optimized_path = []
        self.exploration_path = [(1, 1)]
        self.current_logic_pos = (1, 1)
        
        self.run_start_time = None
        self.exploration_duration = 0.0
        self.fast_run_duration = 0.0
        self.fast_run_start_time = None
        
        if not keep_benchmark:
            self.benchmark_mode = False
            self.current_benchmark_run = 0
            self.fast_run_times = []
            
        from config import WALL_THICK, PATH_WIDTH
        start_x_phys = float(WALL_THICK + PATH_WIDTH / 2)
        start_y_phys = float(WALL_THICK + PATH_WIDTH / 2)
        self.pos_x, self.pos_y, self.heading = start_x_phys, start_y_phys, 0.0
        self.min_seen_x, self.max_seen_x = start_x_phys, start_x_phys
        self.min_seen_y, self.max_seen_y = start_y_phys, start_y_phys
        self.last_vx, self.last_vy = 0.0, 0.0
        
        self.max_logic_r = 1
        self.max_logic_c = 1
        self.discovered_n = None
        self.discovered_m = None
        self.computed_goal = None
        self.return_path = []
        self.exploration_done = False
        self.finalizing = False

controller = DFSController()

def expand_map_if_needed(phys_x, phys_y, controller):
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

command_queue = queue.Queue()

async def client_session(ws):
    nonce = json.loads(await ws.recv())["nonce"]
    await ws.send(json.dumps({"auth": hmac.new(bytes.fromhex(KEY_HEX), bytes.fromhex(nonce), hashlib.sha256).hexdigest()}))
    if json.loads(await ws.recv()).get("auth") != "ok":
        return

    log.info("Połączono asynchronicznie z silnikiem fizycznym drona.")

    async for msg in ws:
        data = json.loads(msg)
        has_laser = "laser" in data

        if "maze_id" in data:
            if data["maze_id"] < controller.current_maze_id:
                continue
            elif data["maze_id"] > controller.current_maze_id:
                controller.current_maze_id = data["maze_id"]
                controller.reset_logic_and_maps(keep_benchmark=controller.benchmark_mode)

        if "phys_maze" in data:
            controller.phys_maze_data = data["phys_maze"]

        is_at_meta = data.get("at_meta", False)
        if controller.teleporting:
            if not is_at_meta:
                controller.teleporting = False
                controller.current_logic_pos = (1, 1)
                controller.stuck_frames = 0
                controller.fast_run_start_time = time.time()
            elif not has_laser:
                continue

        prev_px, prev_py = controller.pos_x, controller.pos_y
        if "pos_x" in data:
            if not hasattr(controller, 'last_raw_x'):
                controller.last_raw_x = data["pos_x"]
                controller.last_raw_y = data["pos_y"]
            
            dx_odom = data["pos_x"] - controller.last_raw_x
            dy_odom = data["pos_y"] - controller.last_raw_y
            
            controller.last_raw_x = data["pos_x"]
            controller.last_raw_y = data["pos_y"]
            
            controller.pos_x += dx_odom
            controller.pos_y += dy_odom
            controller.heading = data.get("heading", controller.heading)

        if not has_laser:
            try:
                cmd_from_dashboard = command_queue.get_nowait()
                if cmd_from_dashboard == "reset":
                    await ws.send(json.dumps({"cmd": "reset"}))
                    command_queue.task_done()
                    continue
                elif isinstance(cmd_from_dashboard, dict):
                    await ws.send(json.dumps(cmd_from_dashboard))
                    command_queue.task_done()
                    continue
            except queue.Empty:
                pass
            await ws.send(json.dumps(controller.get_command()))
            continue

        if controller.target_logic is not None and not controller.finished:
            dist_moved = math.hypot(data["pos_x"] - prev_px, data["pos_y"] - prev_py)
            if dist_moved < 0.05:
                controller.stuck_frames += 1
            else:
                controller.stuck_frames = 0

            if controller.stuck_frames >= 20:
                r1, c1 = controller.current_logic_pos
                r2, c2 = controller.target_logic
                mid_r, mid_c = (r1 + r2) // 2, (c1 + c2) // 2
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

        if controller.target_logic is not None and not controller.fast_run:
            r1, c1 = controller.current_logic_pos
            r2, c2 = controller.target_logic
            mid_r, mid_c = (r1 + r2) // 2, (c1 + c2) // 2
            if 0 <= mid_r < LOGIC_ROWS and 0 <= mid_c < LOGIC_COLS:
                if controller.logic_map[mid_r][mid_c] == 1:
                    if controller.target_logic in controller.visited:
                        controller.visited.remove(controller.target_logic)
                    controller.target_logic = None
                    controller.target_phys = None
                    controller.stuck_frames = 0

        if controller.exploration_done and not controller.fast_run and not controller.finalizing:
            controller._finalize_exploration()
            await ws.send(json.dumps({"cmd": "teleport_to_start"}))
            continue

        if controller.fast_run and controller.finished and not controller.teleporting:
            if controller.fast_run_start_time is not None:
                controller.fast_run_duration = time.time() - controller.fast_run_start_time

            if controller.benchmark_mode:
                controller.fast_run_times.append(controller.fast_run_duration)
                if controller.current_benchmark_run < 50:
                    controller.current_benchmark_run += 1
                    controller.fast_run = False  
                    await ws.send(json.dumps({"cmd": "reset"}))
                    continue
                else:
                    controller.benchmark_mode = False
                    controller.fast_run = False
                    controller.exploration_done = False
                    controller.target_logic = None
                    controller.target_phys = None
                    await ws.send(json.dumps({"vx": 0.0, "vy": 0.0, "w": 0.0}))
                    continue
            else:
                controller.fast_run = False
                controller.exploration_done = False
                controller.target_logic = None
                controller.target_phys = None
                controller.last_vx = 0.0
                controller.last_vy = 0.0
                await ws.send(json.dumps({"vx": 0.0, "vy": 0.0, "w": 0.0}))
                continue

        corner_cut_threshold = 18.0 if controller.fast_run else 4.5

        if controller.target_phys is not None:
            tx, ty = controller.target_phys
            dist_to_target = math.hypot(tx - data["pos_x"], ty - data["pos_y"])
            if dist_to_target < corner_cut_threshold:
                if not controller.fast_run and controller.target_logic is not None:
                    controller.exploration_path.append(controller.target_logic)
                    controller.track_bounding_box(controller.target_logic)
                controller.current_logic_pos = controller.target_logic
                controller.target_logic = None
                controller.target_phys = None

        if controller.target_logic is None and not controller.finished:
            if not controller.fast_run:
                nl = phys_to_logic(data["pos_x"], data["pos_y"])
                if nl != controller.current_logic_pos:
                    controller.track_bounding_box(nl)
                controller.current_logic_pos = nl
            controller.update_target()

        try:
            cmd_from_dashboard = command_queue.get_nowait()
            if cmd_from_dashboard == "reset":
                await ws.send(json.dumps({"cmd": "reset"}))
                command_queue.task_done()
                continue
            elif isinstance(cmd_from_dashboard, dict):
                if cmd_from_dashboard.get("cmd") == "set_meta":
                    controller.meta_placement = cmd_from_dashboard.get("value", "corner")
                await ws.send(json.dumps(cmd_from_dashboard))
                command_queue.task_done()
                continue
        except queue.Empty:
            pass

        await ws.send(json.dumps(controller.get_command()))

async def client_main():
    retry_delay = 1.0
    while True:
        try:
            async with websockets.connect(ROBOT_WS, ping_interval=None, ping_timeout=None) as ws:
                retry_delay = 1.0  
                await client_session(ws)
        except Exception:
            await asyncio.sleep(retry_delay)
            retry_delay = min(retry_delay * 1.5, 5.0)

def start_client():
    asyncio.run(client_main())

if not any(t.name == "ClientWSThread" for t in threading.enumerate()):
    threading.Thread(target=start_client, name="ClientWSThread", daemon=True).start()