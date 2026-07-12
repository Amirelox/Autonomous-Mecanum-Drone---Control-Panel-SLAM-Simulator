import streamlit as st
import json
import numpy as np
import cv2
import math
import time as tm # Dodany import czystego modułu czasu do pomiarów live
from streamlit_autorefresh import st_autorefresh

# Safely bridge into localized thread contexts
import client
from client import controller, prob_map
from config import ROBOT_W_WIDTH, ROBOT_L_LENGTH

st.set_page_config(page_title="Mecanum Drone Dashboard", layout="wide")
st_autorefresh(interval=100, limit=100000, key="frameratesetter")

if "dfs_running" not in st.session_state:
    st.session_state.dfs_running = False

# --- ZAPOBIEGANIE SPAMOWANIU KOLEJKI POLECEŃ SIECIOWYCH ---
if "last_sim_speed" not in st.session_state:
    st.session_state.last_sim_speed = 1.0
if "last_meta_placement" not in st.session_state:
    st.session_state.last_meta_placement = "Narożnik"

st.title("🛸 Autonomous Mecanum Drone - Operations Dashboard")

# ============================================================
# BOCZNY PANEL KONFIGURACJI ŚRODOWISKA (SIDEBAR)
# ============================================================
st.sidebar.header("⚙️ Parametry Środowiska")
sim_speed = st.sidebar.slider("Prędkość symulacji (SIM_SPEED)", 1.0, 10.0, st.session_state.last_sim_speed, step=0.5)
meta_placement = st.sidebar.selectbox("Pozycja mety", ["Narożnik", "Środek"], index=0 if st.session_state.last_meta_placement == "Narożnik" else 1)

if sim_speed != st.session_state.last_sim_speed:
    st.session_state.last_sim_speed = sim_speed
    from client import command_queue
    command_queue.put({"cmd": "set_speed", "value": sim_speed})

if meta_placement != st.session_state.last_meta_placement:
    st.session_state.last_meta_placement = meta_placement
    from client import command_queue
    command_queue.put({"cmd": "set_meta", "value": "corner" if meta_placement == "Narożnik" else "center"})

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
    
    controller.fast_run = False
    controller.teleporting = False
    controller.optimized_path = []
    controller.exploration_path = [(1, 1)]
    
    # RESET INDYWIDUALNYCH STOPERÓW W PANELU
    controller.run_start_time = None
    controller.exploration_duration = 0.0
    controller.fast_run_duration = 0.0
    controller.fast_run_start_time = None
    
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
        
        g_size = int(ROBOT_W_WIDTH)
        gx_phys, gy_phys = client.logic_to_phys(controller.goal_cell[0], controller.goal_cell[1])
        
        cv2.rectangle(img_real, 
                      (int(gx_phys - g_size//2), int(gy_phys - g_size//2)), 
                      (int(gx_phys + g_size//2), int(gy_phys + g_size//2)), 
                      (255, 0, 0), -1)
        
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
        
        # AKTUALIZACJA: Wymuszenie startu stopera głównego przejazdu
        controller.run_start_time = tm.time()
        controller.exploration_duration = 0.0
        controller.fast_run_duration = 0.0
        controller.fast_run_start_time = None
        
        controller.update_target()
        st.success("DFS Solver Activated")
        
    if st.button("📊 START BENCHMARK MODE (50 RUNS)", use_container_width=True):
        controller.benchmark_mode = True
        controller.current_benchmark_run = 1
        controller.fast_run_times = []
        
        # AKTUALIZACJA: Wsparcie stopera również dla startu maratonu
        controller.run_start_time = tm.time()
        controller.exploration_duration = 0.0
        controller.fast_run_duration = 0.0
        
        controller.reset_logic_and_maps(keep_benchmark=True)
        from client import command_queue
        command_queue.put("reset")
        st.success("Zainicjalizowano Tryb Benchmarku. Rozpoczynam maraton 50 labiryntów...")

    if st.button("🛑 STOP / EMERGENCY", type="primary", use_container_width=True):
        st.session_state.dfs_running = False
        controller.finished = True
        controller.benchmark_mode = False
        st.warning("E-Stop Engaged")

    if st.button("🔄 GENERATE NEW MAZE & RESET LOGIC", use_container_width=True):
        controller.phys_maze_data = None 
        reset_explorer()
        st.info("Ecosystem re-generated. New maze is active!")

    # ============================================================
    # NOWOŚĆ: MODUŁ STOPERA LIVE DLA PRZEJAZDÓW INDYWIDUALNYCH
    # ============================================================
    st.markdown("---")
    st.subheader("⏱️ Chronometr Przejazdu")
    
    if not controller.finished:
        if not controller.fast_run:
            # Faza Eksploracji trwa - licznik bije live
            if getattr(controller, 'run_start_time', None) is not None:
                elapsed_expl = tm.time() - controller.run_start_time
                st.metric(label="🔍 Eksploracja DFS (LIVE)", value=f"{elapsed_expl:.2f} s")
            else:
                st.metric(label="🔍 Eksploracja DFS", value="0.00 s")
        else:
            # Faza Fast Run trwa - licznik bije live
            if getattr(controller, 'fast_run_start_time', None) is not None:
                elapsed_fast = tm.time() - controller.fast_run_start_time
                st.metric(label="⚡ Bieg Finałowy (LIVE)", value=f"{elapsed_fast:.2f} s")
            else:
                st.metric(label="⚡ Bieg Finałowy", value="0.00 s")
    else:
        # Przejazd zatrzymany lub zakończony - zamrażamy i pokazujemy ostatnie zapisane wyniki
        t_col1, t_col2 = st.columns(2)
        with t_col1:
            st.metric(label="🏁 Ostatni DFS", value=f"{getattr(controller, 'exploration_duration', 0.0):.2f} s")
        with t_col2:
            st.metric(label="🏆 Ostatni Fast Run", value=f"{getattr(controller, 'fast_run_duration', 0.0):.2f} s")

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

# ============================================================
# LIVE BENCHMARK MODE STATISTICS DISPLAY
# ============================================================
if controller.benchmark_mode or controller.fast_run_times:
    st.markdown("---")
    st.header("📈 Maraton Wyników – Analityka Statystyczna")
    
    progress = len(controller.fast_run_times)
    st.write(f"**Postęp maratonu:** Zrealizowano **{progress} z 50** pełnych cykli przejazdów.")
    
    if controller.fast_run_times:
        times = controller.fast_run_times
        
        m_col1, m_col2, m_col3, m_col4 = st.columns(4)
        with m_col1:
            st.metric(label="Średni czas Fast Run", value=f"{np.mean(times):.3f} s")
        with m_col2:
            st.metric(label="🚀 Najszybszy (Min)", value=f"{np.min(times):.3f} s")
        with m_col3:
            st.metric(label="🐢 Najwolniejszy (Max)", value=f"{np.max(times):.3f} s")
        with m_col4:
            st.metric(label="📉 Odchylenie std (σ)", value=f"{np.std(times):.3f} s")
            
        st.subheader("Wykres dystrybucji czasów w kolejnych próbach")
        st.line_chart(times)