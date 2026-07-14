from nicegui import ui, app
import json
import numpy as np
import cv2
import math
import time
import base64

import client
from client import controller, prob_map
from config import ROBOT_W_WIDTH, ROBOT_L_LENGTH, CELL_SIZE, WALL_THICK, PATH_WIDTH

DISPLAY_SIZE = (450, 450)

# Pusta klatka startowa dla interactive_image (ciemnoszary, 450×450)
_blank_frame = np.full((*DISPLAY_SIZE, 3), 38, dtype=np.uint8)
_, _buf = cv2.imencode('.jpg', _blank_frame)
_BLANK_B64 = base64.b64encode(_buf).decode('utf-8')

# WYMUSZENIE CIEMNEGO MOTYWU GLOBALNIE
dark = ui.dark_mode()
dark.enable()

def reset_explorer():
    controller.finished = True
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
    controller.max_logic_r = 1
    controller.max_logic_c = 1
    controller.goal_cell = None
    controller.exploration_done = False
    controller.fast_run = False
    controller.teleporting = False
    controller.optimized_path = []
    controller.exploration_path = [(1, 1)]
    
    controller.exploration_start_time = None
    controller.fast_run_start_time = None
    controller.exploration_elapsed = 0.0
    controller.fast_run_elapsed = 0.0
    
    start_x_phys = float(WALL_THICK + PATH_WIDTH / 2)
    start_y_phys = float(WALL_THICK + PATH_WIDTH / 2)
    controller.pos_x, controller.pos_y, controller.heading = start_x_phys, start_y_phys, 0.0
    controller.min_seen_x, controller.max_seen_x = start_x_phys, start_x_phys
    controller.min_seen_y, controller.max_seen_y = start_y_phys, start_y_phys
    controller.last_vx = 0.0
    controller.last_vy = 0.0
    
    from client import command_queue
    command_queue.put("reset")

def encode_image_to_base64(img):
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 75]
    _, buffer = cv2.imencode('.jpg', img, encode_param)
    return base64.b64encode(buffer).decode('utf-8')

def render_real_world():
    if controller.phys_maze_data is None:
        return None
    
    real_maze = np.array(controller.phys_maze_data)
    img_real = np.zeros((real_maze.shape[0], real_maze.shape[1], 3), dtype=np.uint8)
    img_real[real_maze == 0] = [38, 38, 38]
    img_real[real_maze == 1] = [255, 255, 255]
    
    g_size = int(ROBOT_W_WIDTH)
    if controller.goal_cell is not None:
        gx, gy = client.logic_to_phys(controller.goal_cell[0], controller.goal_cell[1])
        goal_x_pixels, goal_y_pixels = int(gx), int(gy)
    else:
        goal_x_pixels = img_real.shape[1] - 16
        goal_y_pixels = img_real.shape[0] - 16
    
    cv2.rectangle(img_real, (int(goal_x_pixels - g_size//2), int(goal_y_pixels - g_size//2)), 
                  (int(goal_x_pixels + g_size//2), int(goal_y_pixels + g_size//2)), (255, 0, 0), -1)
    
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
    rw, rl = int(ROBOT_W_WIDTH), int(ROBOT_L_LENGTH)
    if 0 <= ry < img_real.shape[0] and 0 <= rx < img_real.shape[1]:
        cv2.rectangle(img_real, (rx - rw//2, ry - rl//2), (rx + rw//2, ry + rl//2), (30, 144, 255), -1)
    
    return cv2.resize(img_real, DISPLAY_SIZE, interpolation=cv2.INTER_NEAREST)

def render_slam_map():
    safe_prob_map = client.prob_map.copy()
    img_gray = (1.0 - safe_prob_map) * 255
    img_gray = np.clip(img_gray, 0, 255).astype(np.uint8)
    img_rgb = cv2.cvtColor(img_gray, cv2.COLOR_GRAY2RGB)
    
    rx, ry = int(controller.pos_x), int(controller.pos_y)
    rw, rl = int(ROBOT_W_WIDTH), int(ROBOT_L_LENGTH)
    if 0 <= ry < img_rgb.shape[0] and 0 <= rx < img_rgb.shape[1]:
        cv2.rectangle(img_rgb, (rx - rw//2, ry - rl//2), (rx + rw//2, ry + rl//2), (30, 144, 255), -1)
    
    margin = 60
    min_x = max(0, int(controller.min_seen_x - margin))
    max_x = min(client.PHYS_W, int(controller.max_seen_x + margin)) 
    min_y = max(0, int(controller.min_seen_y - margin))
    max_y = min(client.PHYS_H, int(controller.max_seen_y + margin)) 
    
    if max_x > min_x and max_y > min_y:
        cropped_map = img_rgb[min_y:max_y, min_x:max_x]
        return cv2.resize(cropped_map, DISPLAY_SIZE, interpolation=cv2.INTER_NEAREST)
    return cv2.resize(img_rgb, DISPLAY_SIZE, interpolation=cv2.INTER_NEAREST)

# ============================================================
# LAYOUT NICEGUI (Ciemny motyw, lewa/prawa strona)
# ============================================================

# CSS do eliminacji migotania obrazków
ui.add_head_html('''
<style>
.smooth-image {
    transition: opacity 0.05s linear;
}
.smooth-image.loading {
    opacity: 0.7;
}
</style>
<script>
function smoothImageUpdate(imgElement, newSrc) {
    if (!imgElement || !newSrc) return;
    imgElement.classList.add('loading');
    const tempImg = new Image();
    tempImg.onload = function() {
        imgElement.src = newSrc;
        imgElement.classList.remove('loading');
    };
    tempImg.src = newSrc;
}
</script>
''')

ui.page_title("Mecanum Drone Dashboard")

with ui.header().classes('bg-blue-900 text-white items-center'):
    ui.label('🛸 Autonomous Mecanum Drone - Operations Dashboard').classes('text-h5')

with ui.column().classes('w-full p-4 gap-4'):
    
    # RZĄD 1: Dwa okna obok siebie (Lewa i Prawa strona)
    with ui.row().classes('w-full gap-4'):
        # LEWA STRONA
        with ui.card().classes('flex-1 min-w-0 bg-gray-800 text-white p-4'):
            ui.label('🌍 Simulator - Real World').classes('text-h5 mb-2')
            with ui.element('div').classes('w-full flex items-center justify-center bg-gray-900 rounded border border-gray-700'):
                img_real = ui.html(f'<img id="img-real" src="data:image/jpeg;base64,{_BLANK_B64}" class="max-w-full rounded smooth-image" style="width:100%;height:auto;">')
            ui.label('Ground truth state vector layout (Live Lasers)').classes('text-caption text-gray-400 mt-2')
        
        # PRAWA STRONA
        with ui.card().classes('flex-1 min-w-0 bg-gray-800 text-white p-4'):
            ui.label('🗺️ Client - SLAM Grid Map').classes('text-h5 mb-2')
            with ui.element('div').classes('w-full flex items-center justify-center bg-gray-900 rounded border border-gray-700'):
                img_slam = ui.html(f'<img id="img-slam" src="data:image/jpeg;base64,{_BLANK_B64}" class="max-w-full rounded smooth-image" style="width:100%;height:auto;">')
            ui.label('Discovered tracking map (Live Auto-Zoom)').classes('text-caption text-gray-400 mt-2')

    # RZĄD 2: Timery i kontrola prędkości
    with ui.card().classes('w-full bg-gray-800 text-white p-4'):
        with ui.row().classes('w-full gap-8 items-center'):
            with ui.column():
                ui.label('⏱️ Czas Eksploracji').classes('text-h6 text-gray-300')
                exploration_time_label = ui.label('0.00s').classes('text-h4 text-green-400 font-mono')
            with ui.column():
                ui.label('⏱️ Czas Fast Run').classes('text-h6 text-gray-300')
                fast_run_time_label = ui.label('0.00s').classes('text-h4 text-blue-400 font-mono')
            with ui.column().classes('flex-1'):
                ui.label('🎬 Prędkość Animacji').classes('text-h6 text-gray-300 mb-2')
                speed_slider = ui.slider(min=1, max=5, value=1, step=1).classes('w-full')
                speed_label = ui.label('1x (bazowa)').classes('text-sm text-gray-400 mt-1')
                
                def on_speed_change(e):
                    speed_val = e.value
                    # Bazowy interwał 0.15s, dzielimy przez speed_val
                    interval = 0.15 / speed_val
                    # Ograniczamy minimalny interwał do 30ms (~33 FPS) aby uniknąć lagów
                    interval = max(interval, 0.03)
                    update_timer.cancel()
                    globals()['update_timer'] = ui.timer(interval, update_frames)
                    speed_label.set_text(f'{speed_val}x' + (' (wolno)' if speed_val == 1 else ' (normalna)' if speed_val == 2 else ' (szybko)' if speed_val <= 3 else ' (bardzo szybko)'))
                
                speed_slider.on_value_change(on_speed_change)

    # RZĄD 3: Sterowanie i API
    with ui.row().classes('w-full gap-4'):
        with ui.card().classes('w-1/3 bg-gray-800 text-white p-4'):
            ui.label('🤖 Autonomy Control').classes('text-h6 mb-4')
            
            meta_choice = ui.select(
                ['📐 Przeciwległy Narożnik', '🎯 Geometryczny Środek'],
                label='📍 Lokalizacja punktu STOP (Meta)',
                value='📐 Przeciwległy Narożnik'
            ).classes('w-full mb-4 bg-gray-700 text-white')
            
            def on_meta_change(e):
                chosen_mode = "corner" if "Narożnik" in e.value else "center"
                controller.meta_placement = chosen_mode
                client.command_queue.put({"meta_placement": chosen_mode})
            meta_choice.on_value_change(on_meta_change)
            
            def start_dfs():
                controller.finished = False
                controller.last_vx = 0.0
                controller.last_vy = 0.0
                controller.exploration_start_time = time.time()
                controller.update_target()
                ui.notify('DFS Solver Activated', type='positive')
            
            def stop_dfs():
                controller.finished = True
                ui.notify('E-Stop Engaged', type='warning')
            
            def reset_maze():
                controller.phys_maze_data = None
                reset_explorer()
                ui.notify('Ecosystem re-generated. New maze is active!', type='info')
            
            ui.button('🚀 START DFS RUN', on_click=start_dfs).props('color=positive').classes('w-full mb-2')
            ui.button('🛑 STOP / EMERGENCY', on_click=stop_dfs).props('color=negative').classes('w-full mb-2')
            ui.button('🔄 GENERATE NEW MAZE & RESET', on_click=reset_maze).classes('w-full bg-gray-700 text-white')

        with ui.card().classes('w-2/3 bg-gray-800 text-white p-4'):
            ui.label('💻 Manual Mecanum JSON API Console').classes('text-h6 mb-4')
            user_command = ui.input('Mecanum JSON API Payload:', value='{"vx": 0.0, "vy": 0.6, "w": 0.0}').classes('w-full mb-4 bg-gray-700 text-white')
            
            def inject_command():
                try:
                    parsed_cmd = json.loads(user_command.value)
                    if not controller.finished:
                        controller.last_vx = parsed_cmd.get("vx", 0.0)
                        controller.last_vy = parsed_cmd.get("vy", 0.0)
                    ui.notify('Payload sent to control register', type='positive')
                except json.JSONDecodeError:
                    ui.notify('Invalid JSON syntax provided', type='negative')
            
# Timer aktualizacji (150ms = stabilne ~6.6 FPS, bez migotania)
def update_frames():
    frame_real = render_real_world()
    if frame_real is not None:
        b64_real = encode_image_to_base64(frame_real)
        ui.run_javascript(f'smoothImageUpdate(document.getElementById("img-real"), "data:image/jpeg;base64,{b64_real}")')
    
    frame_slam = render_slam_map()
    if frame_slam is not None:
        b64_slam = encode_image_to_base64(frame_slam)
        ui.run_javascript(f'smoothImageUpdate(document.getElementById("img-slam"), "data:image/jpeg;base64,{b64_slam}")')
    
    if controller.exploration_start_time is not None and not controller.exploration_done:
        exploration_time_label.set_text(f'{time.time() - controller.exploration_start_time:.2f}s')
    elif controller.exploration_elapsed > 0:
        exploration_time_label.set_text(f'{controller.exploration_elapsed:.2f}s (zakończono)')
    
    if controller.fast_run_start_time is not None and not controller.finished:
        fast_run_time_label.set_text(f'{time.time() - controller.fast_run_start_time:.2f}s')
    elif controller.fast_run_elapsed > 0:
        fast_run_time_label.set_text(f'{controller.fast_run_elapsed:.2f}s (zakończono)')

# Timer aktualizacji - zdefiniowany po layout, żeby mieć dostęp do wszystkich zmiennych
update_timer = ui.timer(0.15, update_frames)

ui.run(port=8080, reload=False, title='Mecanum Drone Dashboard')