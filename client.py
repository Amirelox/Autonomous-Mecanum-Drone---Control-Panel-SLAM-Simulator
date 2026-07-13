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

# Import centralized configuration constants
from config import (
    N, M, CELL_SIZE, WALL_THICK, PATH_WIDTH,
    ROBOT_W_WIDTH, ROBOT_L_LENGTH, SENSOR_RADIUS, 
    SENSOR_RANGE, NUM_SENSORS, RAYS_PER_SENSOR, 
    SENSOR_ANGLES_DEG, ray_angles_deg
)

# ============================================================
# SLAM INTERNAL RESOLUTION MAP CONFIG
# ============================================================
LOGIC_ROWS, LOGIC_COLS = 41, 41
PHYS_H, PHYS_W = 500, 500
GOAL_CELL = (2 * (N - 1) + 1, 2 * (M - 1) + 1)

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

        # --- REJESTRACJA DYNAMICZNEJ METS ---
        self.goal_cell = (2 * (N - 1) + 1, 2 * (M - 1) + 1)

        # --- FAST RUN STATE ---
        self.fast_run = False
        self.teleporting = False
        self.current_maze_id = 1
        self.optimized_path = []
        self.exploration_path = [self.start]

        # --- NIEZALEŻNY SYSTEM STOPERA DLA KAŻDEGO BIEGU ---
        self.run_start_time = None
        self.exploration_duration = 0.0
        self.fast_run_duration = 0.0
        self.fast_run_start_time = None

        # --- MODE BENCHMARK COUNTERS ---
        self.benchmark_mode = False
        self.current_benchmark_run = 0
        self.fast_run_times = []

        # === 🟡 BLIND AUTONOMY: BOUNDING BOX & POST-PROCESSING ===
        self.max_logic_r = 1          # furthest explored row (logic coords)
        self.max_logic_c = 1          # furthest explored col (logic coords)
        self.meta_placement = "corner" # "corner" or "center" — set by dashboard
        self.discovered_n = None      # computed maze height after exploration
        self.discovered_m = None      # computed maze width after exploration
        self.computed_goal = None     # locally computed goal cell (logic coords)
        self.return_path = []         # BFS path to navigate back to (1,1)
        self.exploration_done = False # set True when fully explored + back at start
        self.finalizing = False       # prevents re-entering finalize

        self._lock = threading.Lock()

    def _locked(self):
        return self._lock

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
                for d in range(1, int(c_range), 2):
                    cy, cx = int(sy + dy*d), int(sx + dx*d)
                    if 0 <= cy < PHYS_H and 0 <= cx < PHYS_W:
                        prob_map[cy, cx] = max(0.0, prob_map[cy, cx] - 0.05)

                if c_range >= 1:
                    for d in range(0, int(max(0.0, c_range - 12.0)), 2):
                        r, c = phys_to_logic(sx + dx*d, sy + dy*d)
                        if 0 <= r < LOGIC_ROWS and 0 <= c < LOGIC_COLS and self.logic_map[r][c] == -1:
                            self.logic_map[r][c] = 0

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

    def track_bounding_box(self, logic_pos):
        """Update max explored extent when entering a new logic cell."""
        r, c = logic_pos
        if r > self.max_logic_r:
            self.max_logic_r = r
        if c > self.max_logic_c:
            self.max_logic_c = c

    def compute_shortest_path(self, start, goal):
        """Generates the absolute shortest path using Breadth-First Search (BFS)."""
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
        """Post-processing after blind exploration: compute maze size, goal, BFS path."""
        if self.finalizing:
            return
        self.finalizing = True

        # Capture exploration duration
        if self.run_start_time is not None:
            self.exploration_duration = time.time() - self.run_start_time

        # 1. Compute discovered maze dimensions
        self.discovered_n = (self.max_logic_r + 1) // 2
        self.discovered_m = (self.max_logic_c + 1) // 2
        log.info(f"🟡 Discovered maze: {self.discovered_n}×{self.discovered_m} "
                 f"(max_logic_r={self.max_logic_r}, max_logic_c={self.max_logic_c})")

        # 2. Compute goal cell based on placement mode
        if self.meta_placement == "corner":
            goal_r = self.max_logic_r
            goal_c = self.max_logic_c
        else:  # "center"
            goal_r = 2 * (self.discovered_n // 2) + 1
            goal_c = 2 * (self.discovered_m // 2) + 1
        self.computed_goal = (goal_r, goal_c)
        # Also set goal_cell so dashboard renders correctly
        self.goal_cell = (goal_r, goal_c)
        log.info(f"🟡 Computed goal: {self.computed_goal} (mode={self.meta_placement})")

        # 3. Seal the exploration path into logic_map
        for idx in range(len(self.exploration_path)):
            r, c = self.exploration_path[idx]
            self.logic_map[r][c] = 0
            if idx > 0:
                pr, pc = self.exploration_path[idx - 1]
                self.logic_map[(r + pr) // 2][(c + pc) // 2] = 0

        # 4. Mark goal cell as free
        self.logic_map[goal_r][goal_c] = 0
        self.logic_map[(goal_r + 1) // 2][(goal_c + 1) // 2] = 0

        # 5. BFS shortest path from start to goal
        full_path = self.compute_shortest_path(self.start, self.computed_goal)
        self.optimized_path = list(full_path[1:]) if len(full_path) > 1 else []
        log.info(f"🟡 BFS path computed: {len(self.optimized_path)} steps")

        # 6. Transition to speedrun mode
        self.fast_run = True
        self.teleporting = True
        self.target_logic = None
        self.target_phys = None
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.exploration_done = True
        self.finalizing = False
        log.info("🏁 BLIND EXPLORATION COMPLETE! Teleporting to start for speedrun...")

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
            # Try normal DFS first
            nxt = self.find_next_target()
            if nxt:
                self.target_logic = nxt
                self.target_phys = logic_to_phys(nxt[0], nxt[1])
                self.last_vx, self.last_vy = 0.0, 0.0
                return

            # 🟡 BLIND AUTONOMY: path_stack empty — need to return to (1,1)
            if self.current_logic_pos == self.start:
                # Back at start with empty stack → exploration complete!
                self.exploration_done = True
                log.info("🟡 Exploration complete — all cells visited, back at start")
                return

            # Not at start yet — seal exploration path and compute BFS back to (1,1)
            if not self.return_path:
                # First seal the known path so BFS can traverse it
                for idx in range(len(self.exploration_path)):
                    r, c = self.exploration_path[idx]
                    self.logic_map[r][c] = 0
                    if idx > 0:
                        pr, pc = self.exploration_path[idx-1]
                        self.logic_map[(r+pr)//2][(c+pc)//2] = 0

                path_home = self.compute_shortest_path(self.current_logic_pos, self.start)
                if len(path_home) > 1:
                    self.return_path = list(path_home[1:])
                    log.info(f"🟡 Returning to start via BFS ({len(self.return_path)} steps)")
                else:
                    # BFS failed — some cells still unknown (-1). Do a second pass
                    # treating -1 cells as traversable (they're unvisited paths, not walls)
                    from collections import deque
                    dq = deque([self.current_logic_pos])
                    parent = {self.current_logic_pos: None}
                    found = False
                    while dq and not found:
                        cr, cc = dq.popleft()
                        for dr, dc in [(0, 2), (2, 0), (0, -2), (-2, 0)]:
                            nr, nc = cr + dr, cc + dc
                            if not (0 <= nr < LOGIC_ROWS and 0 <= nc < LOGIC_COLS):
                                continue
                            if (nr, nc) in parent:
                                continue
                            mid_r, mid_c = (cr + nr)//2, (cc + nc)//2
                            mid_val = self.logic_map[mid_r][mid_c]
                            tgt_val = self.logic_map[nr][nc]
                            # Can traverse if mid is not a wall and target is not a wall
                            if mid_val != 1 and tgt_val != 1:
                                if mid_val == -1:
                                    self.logic_map[mid_r][mid_c] = 0
                                if tgt_val == -1:
                                    self.logic_map[nr][nc] = 0
                                parent[(nr, nc)] = (cr, cc)
                                dq.append((nr, nc))
                                if (nr, nc) == self.start:
                                    found = True
                                    break
                    if found:
                        # Reconstruct path: follow parent chain from current to start
                        p = []
                        pos = self.current_logic_pos
                        while pos != self.start:
                            pos = parent[pos]
                            p.append(pos)
                        self.return_path = p  # [step1, step2, ..., start]
                        log.info(f"🟡 Returning to start via BFS-soft ({len(self.return_path)} steps)")
                    else:
                        log.warning("🟡 No return path found — forcing exploration complete")
                        self.exploration_done = True
                        return

            if self.return_path:
                nxt = self.return_path.pop(0)
                self.target_logic = nxt
                self.target_phys = logic_to_phys(nxt[0], nxt[1])
                self.last_vx, self.last_vy = 0.0, 0.0

    def get_command(self):
        """Returns motion command. Returns stop immediately when finished."""
        if self.finished or self.target_phys is None:
            return {"vx": 0.0, "vy": 0.0, "w": 0.0}

        tx, ty = self.target_phys
        
        # Safety offset only in speedrun mode — during blind exploration
        # the robot must treat every cell equally (no "goal" known yet)
        if self.fast_run and self.target_logic == self.goal_cell:
            diff_x = tx - self.pos_x
            diff_y = ty - self.pos_y
            safety_offset = 8.0
            
            if abs(diff_x) > abs(diff_y):
                if diff_x > 0: tx -= safety_offset
                else: tx += safety_offset
            else:
                if diff_y > 0: ty -= safety_offset
                else: ty += safety_offset

        real_dx, real_dy = tx - self.pos_x, ty - self.pos_y
        move_speed = 0.60

        # PŁYNNY RUCH OSIOWY
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

    def reset_logic_and_maps(self, keep_benchmark=False):
        """Resets structural SLAM nodes. Safely preserves global benchmark mode sequences."""
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
        
        # RESET LICZNIKÓW CZASU
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
        
        # 🟡 Reset blind autonomy state
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

command_queue = queue.Queue()

async def client_session(ws):
    """Single client session — processes messages until disconnect."""
    nonce = json.loads(await ws.recv())["nonce"]
    await ws.send(json.dumps({"auth": hmac.new(bytes.fromhex(KEY_HEX), bytes.fromhex(nonce), hashlib.sha256).hexdigest()}))
    if json.loads(await ws.recv()).get("auth") != "ok":
        log.error("Authentication failed")
        return

    log.info("Client authenticated successfully")

    async for msg in ws:
        data = json.loads(msg)
        has_laser = "laser" in data

        # 🟡 BLIND: ignore goal_cell from server — compute locally
        # if "goal_cell" in data: controller.goal_cell = tuple(data["goal_cell"])

        if "maze_id" in data:
            if data["maze_id"] < controller.current_maze_id:
                continue
            elif data["maze_id"] > controller.current_maze_id:
                controller.current_maze_id = data["maze_id"]
                controller.reset_logic_and_maps(keep_benchmark=controller.benchmark_mode)
                log.info(f"♻️ Wykryto nowy labirynt (ID: {controller.current_maze_id}). Czyszczenie mapy...")

        if "phys_maze" in data:
            controller.phys_maze_data = data["phys_maze"]

        is_at_meta = data.get("at_meta", False)
        if controller.teleporting:
            # 🟡 BLIND: teleport completes when NOT at meta (i.e. robot moved from start)
            if not is_at_meta:
                controller.teleporting = False
                controller.current_logic_pos = (1, 1)
                controller.stuck_frames = 0
                log.info(f"⚡ Teleportacja udana! Rozpoczynam FAST RUN dla rundy {controller.current_benchmark_run if controller.benchmark_mode else 1}...")
                controller.fast_run_start_time = time.time()
            elif not has_laser:
                continue  # Idle heartbeat while teleporting — skip

        # Always update position from any message (after stall detector reads old pos)
        prev_px, prev_py = controller.pos_x, controller.pos_y
        if "pos_x" in data:
            controller.pos_x = data["pos_x"]
            controller.pos_y = data["pos_y"]
            controller.heading = data.get("heading", controller.heading)

        # Skip SLAM/control processing if no laser data (idle heartbeat)
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
            command = controller.get_command()
            await ws.send(json.dumps(command))
            continue

        # --- STALL DETECTOR MECHANISM (uses prev position before update) ----
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

        # ================================================================
        # 🟡 BLIND AUTONOMY: transition logic
        # ================================================================
        if controller.exploration_done and not controller.fast_run and not controller.finalizing:
            controller._finalize_exploration()
            await ws.send(json.dumps({"cmd": "teleport_to_start"}))
            continue

        # Handle speedrun completion (fast_run finished)
        if controller.fast_run and controller.finished and not controller.teleporting:
            if controller.fast_run_start_time is not None:
                controller.fast_run_duration = time.time() - controller.fast_run_start_time

            if controller.benchmark_mode:
                controller.fast_run_times.append(controller.fast_run_duration)
                log.info(f"📊 [BENCHMARK] Runda {controller.current_benchmark_run}/50 zaliczona! Czas Fast Run: {controller.fast_run_duration:.3f}s")
                if controller.current_benchmark_run < 50:
                    controller.current_benchmark_run += 1
                    await ws.send(json.dumps({"cmd": "reset"}))
                    continue
                else:
                    log.info("🏆 MARATON BENCHMARK ZAKOŃCZONY! Zebrano kompletne statystyki.")
                    controller.benchmark_mode = False
                    controller.target_logic = None
                    controller.target_phys = None
                    await ws.send(json.dumps({"vx": 0.0, "vy": 0.0, "w": 0.0}))
                    continue
            else:
                controller.target_logic = None
                controller.target_phys = None
                controller.last_vx = 0.0
                controller.last_vy = 0.0
                await ws.send(json.dumps({"vx": 0.0, "vy": 0.0, "w": 0.0}))
                log.info("🏆 TRYB INDYWIDUALNY ZAKOŃCZONY SUKCESEM!")
                continue

        # ================================================================
        # OLD is_at_meta logic REMOVED — robot no longer uses server meta flag
        # ================================================================

        if controller.target_phys is not None:
            tx, ty = controller.target_phys
            dist_to_target = math.hypot(tx - data["pos_x"], ty - data["pos_y"])
            if dist_to_target < 5.0:
                # 🟡 Track bounding box on entering new cell
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
                # 🟡 BLIND: intercept meta_placement updates
                if cmd_from_dashboard.get("cmd") == "set_meta":
                    controller.meta_placement = cmd_from_dashboard.get("value", "corner")
                    log.info(f"🟡 Meta placement set to: {controller.meta_placement}")
                await ws.send(json.dumps(cmd_from_dashboard))
                command_queue.task_done()
                continue
        except queue.Empty:
            pass

        command = controller.get_command()
        json_request = json.dumps(command)
        await ws.send(json_request)

        # (File logging removed — use terminal logging instead)


async def client_main():
    """Main client loop with auto-reconnection on disconnect."""
    retry_delay = 1.0
    while True:
        try:
            log.info(f"Connecting to {ROBOT_WS}...")
            async with websockets.connect(ROBOT_WS, ping_interval=None, ping_timeout=None) as ws:
                log.info("Connected to server")
                retry_delay = 1.0  # Reset delay on successful connection
                await client_session(ws)
        except websockets.exceptions.ConnectionClosed as e:
            log.warning(f"Connection closed: {e}. Reconnecting in {retry_delay:.0f}s...")
        except (OSError, asyncio.TimeoutError) as e:
            log.warning(f"Connection error: {e}. Reconnecting in {retry_delay:.0f}s...")
        except Exception as e:
            log.error(f"Unexpected error: {e}. Reconnecting in {retry_delay:.0f}s...")
        
        await asyncio.sleep(retry_delay)
        retry_delay = min(retry_delay * 1.5, 10.0)  # Exponential backoff up to 10s


def start_client():
    """Starts the client event loop in a daemon thread."""
    asyncio.run(client_main())


if not any(t.name == "ClientWSThread" for t in threading.enumerate()):
    threading.Thread(target=start_client, name="ClientWSThread", daemon=True).start()
    log.info("Client thread started")