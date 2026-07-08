import streamlit as st
import json
import numpy as np
import cv2
import math
from streamlit_autorefresh import st_autorefresh

# Safely bridge into localized thread contexts
import client
from client import controller, prob_map
from config import ROBOT_W_WIDTH, ROBOT_L_LENGTH

st.set_page_config(page_title="Mecanum Drone Dashboard", layout="wide")
st_autorefresh(interval=100, limit=100000, key="frameratesetter")

if "dfs_running" not in st.session_state:
    st.session_state.dfs_running = False

st.title("🛸 Autonomous Mecanum Drone - Operations Dashboard")
DISPLAY_SIZE = (450, 450)

def reset_explorer():
    """Wipes memory nodes, flushes SLAM matrices and signs off server command cues."""
    global prob_map
    controller.finished = True
    st.session_state.dfs_running = False
    
    from config import CELL_SIZE, WALL_THICK, PATH_WIDTH
    
    client.PHYS_H, client.PHYS_W = 500, 500
    client.LOGIC_ROWS, client.LOGIC_COLS = 41, 41
    client.prob_map = np.full((500, 500), 0.5)
    
    controller.logic_map = [[-1] * client.LOGIC_COLS for _ in range(client.LOGIC_ROWS)]
    controller.logic_map[1][1] = 0
    controller.visited = {(1, 1)}
    controller.path_stack = []
    controller.target_logic = None
    controller.target_phys = None
    controller.stuck_frames = 0
    controller.current_logic_pos = (1, 1)
    
    start_x_phys = float(WALL_THICK + PATH_WIDTH / 2)
    start_y_phys = float(WALL_THICK + PATH_WIDTH / 2)
    controller.pos_x, controller.pos_y, controller.heading = start_x_phys, start_y_phys, 0.0
    
    controller.min_seen_x, controller.max_seen_x = start_x_phys, start_x_phys
    controller.min_seen_y, controller.max_seen_y = start_y_phys, start_y_phys
    
    controller.last_vx = 0.0
    controller.last_vy = 0.0
    
    from client import command_queue
    command_queue.put("reset")

# ============================================================
# HMI RENDERING LAYOUT: TWO COLS APART
# ============================================================
col1, col2 = st.columns(2)

with col1:
    st.header("🌍 Simulator - Real World")
    if controller.phys_maze_data is not None:
        real_maze = np.array(controller.phys_maze_data)
        img_real = np.zeros((real_maze.shape[0], real_maze.shape[1], 3), dtype=np.uint8)
        img_real[real_maze == 0] = [38, 38, 38]   
        img_real[real_maze == 1] = [255, 255, 255] 
        
        # Plot Goal Target: Render red rectangle at track end boundary
        g_size = int(ROBOT_W_WIDTH)
        goal_x_pixels = img_real.shape[1] - 16
        goal_y_pixels = img_real.shape[0] - 16
        
        cv2.rectangle(img_real, 
                      (int(goal_x_pixels - g_size//2), int(goal_y_pixels - g_size//2)), 
                      (int(goal_x_pixels + g_size//2), int(goal_y_pixels + g_size//2)), 
                      (255, 0, 0), -1)
        
        # Render live LiDAR sensor beam arrays
        if hasattr(controller, 'laser_data') and controller.laser_data:
            from client import NUM_SENSORS, RAYS_PER_SENSOR, SENSOR_ANGLES_DEG, SENSOR_RADIUS, ray_angles_deg
            idx = 0
            for i in range(NUM_SENSORS):
                s_hdg = math.radians(math.degrees(controller.heading) + SENSOR_ANGLES_DEG[i])
                sx = controller.pos_x + SENSOR_RADIUS * math.cos(s_hdg)
                sy = controller.pos_y + SENSOR_RADIUS * math.sin(s_hdg)
                for ang in ray_angles_deg:
                    if idx >= len(controller.laser_data): break
                    r_ang = s_hdg + math.radians(ang)
                    dist = controller.laser_data[idx]["d"]
                    ex, ey = int(sx + math.cos(r_ang) * dist), int(sy + math.sin(r_ang) * dist)
                    cv2.line(img_real, (int(sx), int(sy)), (ex, ey), (255, 50, 50), 1)
                    idx += 1
        
        # Render Blue Rectangle Drone footprint
        rx, ry = int(controller.pos_x), int(controller.pos_y)
        rw = int(ROBOT_W_WIDTH)
        rl = int(ROBOT_L_LENGTH)
        
        if 0 <= ry < img_real.shape[0] and 0 <= rx < img_real.shape[1]:
            cv2.rectangle(img_real, 
                          (rx - rw//2, ry - rl//2), 
                          (rx + rw//2, ry + rl//2), 
                          (30, 144, 255), -1)
            
        view_real = cv2.resize(img_real, DISPLAY_SIZE, interpolation=cv2.INTER_NEAREST)
        st.image(view_real, caption="Ground truth state vector layout (Live Lasers)")
    else:
        st.info("Awaiting telemetry pipelines from server stream...")

with col2:
    st.header("🗺️ Client - SLAM Grid Map")
    
    # Process probabilisitic matrix map to RGB grayscale spectrums
    img_gray = (1.0 - client.prob_map) * 255
    img_gray = np.clip(img_gray, 0, 255).astype(np.uint8)
    img_rgb = cv2.cvtColor(img_gray, cv2.COLOR_GRAY2RGB)
    
    rx, ry = int(controller.pos_x), int(controller.pos_y)
    rw = int(ROBOT_W_WIDTH)
    rl = int(ROBOT_L_LENGTH)
    
    if 0 <= ry < img_rgb.shape[0] and 0 <= rx < img_rgb.shape[1]:
        cv2.rectangle(img_rgb, 
                      (rx - rw//2, ry - rl//2), 
                      (rx + rw//2, ry + rl//2), 
                      (30, 144, 255), -1)
        
    # Crop bounds dynamically to lock an elegant Auto-Zoom view window
    margin = 60
    min_x = max(0, int(controller.min_seen_x - margin))
    max_x = min(client.PHYS_W, int(controller.max_seen_x + margin)) 
    min_y = max(0, int(controller.min_seen_y - margin))
    max_y = min(client.PHYS_H, int(controller.max_seen_y + margin)) 
    
    if max_x > min_x and max_y > min_y:
        cropped_map = img_rgb[min_y:max_y, min_x:max_x]
        view_slam_zoom = cv2.resize(cropped_map, DISPLAY_SIZE, interpolation=cv2.INTER_NEAREST)
        st.image(view_slam_zoom, caption="Discovered tracking map (Live Auto-Zoom)")
    else:
        view_slam_zoom = cv2.resize(img_rgb, DISPLAY_SIZE, interpolation=cv2.INTER_NEAREST)
        st.image(view_slam_zoom, caption="Discovered tracking map (Live Auto-Zoom)")

st.markdown("---")

# ============================================================
# COMMAND CONSOLE SUBPANELS
# ============================================================
col_ctrl, col_api = st.columns([1, 2])

with col_ctrl:
    st.subheader("🤖 Autonomy Control")
    
    if st.button("🚀 START DFS RUN", use_container_width=True):
        st.session_state.dfs_running = True
        controller.finished = False 
        controller.last_vx = 0.0
        controller.last_vy = 0.0
        
        controller.update_target()
        st.success("DFS Solver Activated")
        
    if st.button("🛑 STOP / EMERGENCY", type="primary", use_container_width=True):
        st.session_state.dfs_running = False
        controller.finished = True
        st.warning("E-Stop Engaged")

    if st.button("🔄 GENERATE NEW MAZE & RESET LOGIC", use_container_width=True):
        controller.phys_maze_data = None 
        reset_explorer()
        st.info("Ecosystem re-generated. New maze is active!")

with col_api:
    st.subheader("💻 Manual Mecanum JSON API Console")
    user_command = st.text_input("Mecanum JSON API Payload:", value='{"vx": 0.0, "vy": 0.6, "w": 0.0}')
    
    if st.button("📤 Inject API Request"):
        try:
            parsed_cmd = json.loads(user_command)
            if not st.session_state.dfs_running:
                controller.last_vx = parsed_cmd.get("vx", 0.0)
                controller.last_vy = parsed_cmd.get("vy", 0.0)
            st.success("Payload sent to control register")
        except json.JSONDecodeError:
            st.error("Invalid JSON syntax provided")