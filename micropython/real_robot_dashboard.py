"""
Dashboard dla Rzeczywistego Robota z ESP32-S3
==============================================
NiceGUI dashboard pokazujący mapę SLAM, telemetrię i kontrolę robota.
Działa na porcie 8081 (nie koliduje z ESP32 WebSocket na 8765).
"""

from nicegui import ui, app
import asyncio
import json
import numpy as np
import cv2
import math
import time
import base64
from typing import Optional

# Importuj kontroler z main.py
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))
from main import RobotController, ESP32WebSocketClient, CELL_SIZE, WALL_THICK, PATH_WIDTH

# Konfiguracja
DASHBOARD_PORT = 8081
DISPLAY_SIZE = (450, 450)

# Globalne zmienne
controller: Optional[RobotController] = None
ws_client: Optional[ESP32WebSocketClient] = None
running = False

# Mapa SLAM
slam_map = np.full((100, 100), 0.5)  # Prawdopodobność zajęcia komórki
visited_cells = set()
robot_path = []

def encode_image_to_base64(img):
    """Konwertuje obraz numpy do base64 dla NiceGUI."""
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 85]
    _, buffer = cv2.imencode('.jpg', img, encode_param)
    return base64.b64encode(buffer).decode('utf-8')

def render_slam_map():
    """Renderuje mapę SLAM jako obraz."""
    global slam_map
    
    # Skaluj mapę do rozmiaru wyświetlania
    map_display = cv2.resize(slam_map, DISPLAY_SIZE, interpolation=cv2.INTER_NEAREST)
    
    # Konwertuj prawdopodobieństwo do kolorów
    img = np.zeros((*DISPLAY_SIZE, 3), dtype=np.uint8)
    
    # Nieodwiedzone = szare, odwiedzone wolne = białe, ściany = czarne
    for r in range(DISPLAY_SIZE[0]):
        for c in range(DISPLAY_SIZE[1]):
            prob = map_display[r, c]
            if prob < 0.3:  # Ściana
                img[r, c] = [0, 0, 0]
            elif prob > 0.7:  # Wolna przestrzeń
                img[r, c] = [255, 255, 255]
            else:  # Nieznane
                img[r, c] = [128, 128, 128]
    
    # Narysuj ścieżkę robota
    for i in range(len(robot_path) - 1):
        x1, y1 = robot_path[i]
        x2, y2 = robot_path[i + 1]
        cv2.line(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
    
    # Narysuj pozycję robota
    if controller:
        robot_x = int(controller.pos_x / CELL_SIZE * DISPLAY_SIZE[1] / 100)
        robot_y = int(controller.pos_y / CELL_SIZE * DISPLAY_SIZE[0] / 100)
        robot_x = min(max(robot_x, 0), DISPLAY_SIZE[1] - 1)
        robot_y = min(max(robot_y, 0), DISPLAY_SIZE[0] - 1)
        
        # Trójkąt wskazujący kierunek
        angle = controller.heading
        size = 10
        points = np.array([
            [robot_x + size * math.cos(angle), robot_y + size * math.sin(angle)],
            [robot_x + size * math.cos(angle + 2.5), robot_y + size * math.sin(angle + 2.5)],
            [robot_x + size * math.cos(angle - 2.5), robot_y + size * math.sin(angle - 2.5)]
        ], dtype=np.int32)
        cv2.fillPoly(img, [points], (0, 0, 255))
    
    return encode_image_to_base64(img)

def update_telemetry():
    """Aktualizuje dane telemetryczne w UI."""
    if not controller:
        return
    
    status_text.value = f"Status: {'ARMED' if controller.armed else 'DISARMED'} | E-Stop: {'ON' if controller.estop else 'OFF'}"
    phase_text.value = f"Faza: {'EXPLORATION' if not controller.exploration_done else 'FAST RUN' if controller.fast_run else 'IDLE'}"
    pos_text.value = f"Pozycja: X={controller.pos_x:.1f}, Y={controller.pos_y:.1f}, Heading={math.degrees(controller.heading):.1f}°"
    cells_text.value = f"Odwiedzone komórki: {len(controller.visited_cells)}"
    
    if controller.goal_cell:
        goal_text.value = f"Cel: ({controller.goal_cell[0]}, {controller.goal_cell[1]})"
    else:
        goal_text.value = "Cel: Brak"

async def update_dashboard():
    """Główna pętla aktualizacji dashboardu."""
    global running
    
    while running:
        try:
            if controller and ws_client and ws_client.connected:
                # Aktualizuj sensory
                controller.update_sensors()
                
                # Wykonaj krok eksploracji
                controller.run_exploration_step()
                
                # Aktualizuj mapę SLAM (placeholder - gdy API będzie gotowe)
                update_slam_from_sensors()
                
                # Dodaj pozycję do ścieżki
                robot_path.append((
                    controller.pos_x / CELL_SIZE * DISPLAY_SIZE[1] / 100,
                    controller.pos_y / CELL_SIZE * DISPLAY_SIZE[0] / 100
                ))
                
                # Aktualizuj UI
                update_telemetry()
                
                # Aktualizuj obraz mapy
                map_b64 = render_slam_map()
                map_image.set_source(f'data:image/jpeg;base64,{map_b64}')
            
            await asyncio.sleep(0.1)  # 10 Hz update
        except Exception as e:
            print(f"[DASHBOARD] Error in update loop: {e}")
            await asyncio.sleep(1)

def update_slam_from_sensors():
    """Aktualizuje mapę SLAM na podstawie danych z czujników."""
    if not controller:
        return
    
    # TODO: Gdy API czujników będzie gotowe, użyj prawdziwych danych
    # Na razie symulujemy aktualizację mapy
    
    # Aktualizuj komórkę当前位置 jako odwiedzoną
    cell_r = int(controller.pos_y / CELL_SIZE)
    cell_c = int(controller.pos_x / CELL_SIZE)
    
    if 0 <= cell_r < 100 and 0 <= cell_c < 100:
        visited_cells.add((cell_r, cell_c))
        slam_map[cell_r, cell_c] = 0.9  # Wolna przestrzeń
        
        # Oznacz sąsiadów jako prawdopodobnie wolne
        for dr in [-1, 0, 1]:
            for dc in [-1, 0, 1]:
                nr, nc = cell_r + dr, cell_c + dc
                if 0 <= nr < 100 and 0 <= nc < 100:
                    if (nr, nc) not in visited_cells:
                        slam_map[nr, nc] = max(slam_map[nr, nc], 0.6)

def start_exploration():
    """Rozpoczyna eksplorację DFS."""
    global controller
    
    if not controller:
        ui.notify('Kontroler nie jest połączony!', type='negative')
        return
    
    if not controller.armed:
        ui.notify('Najpierw uzbrój robota!', type='warning')
        return
    
    controller.finished = False
    controller.exploration_done = False
    controller.fast_run = False
    controller.visited_cells.clear()
    controller.path_stack.clear()
    controller.current_cell = (0, 0)
    controller.visited_cells.add((0, 0))
    
    ui.notify('Eksploracja DFS rozpoczęta!', type='positive')

def stop_robot():
    """Zatrzymuje robota (E-Stop)."""
    global controller
    
    if controller:
        controller.emergency_stop()
        ui.notify('E-STOP Aktywowany!', type='negative')

def arm_robot():
    """Uzbraja robota."""
    global controller, ws_client
    
    if not ws_client or not ws_client.connected:
        ui.notify('Brak połączenia z ESP32!', type='negative')
        return
    
    # Wyślij komendy neutralne aby uzbroić
    async def send_arm_commands():
        for _ in range(10):
            await ws_client.send_command({"vx": 0.0, "vy": 0.0, "w": 0.0})
            await asyncio.sleep(0.05)
        controller.arm()
        ui.notify('Robot uzbrojony!', type='positive')
    
    asyncio.create_task(send_arm_commands())

def disarm_robot():
    """Rozbraja robota."""
    global controller
    
    if controller:
        controller.disarm()
        ui.notify('Robot rozbrojony', type='info')

async def connect_to_esp32():
    """Łączy się z ESP32 i rozpoczyna automatyczną eksplorację."""
    global controller, ws_client, running
    
    try:
        # Utwórz klienta WebSocket
        ws_client = ESP32WebSocketClient(
            host="192.168.4.1",
            port=8765,
            secret_key="robot_secret_2024"
        )
        
        ui.notify('Łączenie z ESP32...', type='info')
        
        # Połącz z ESP32
        if not await ws_client.connect():
            ui.notify('Nie można połączyć się z ESP32!', type='negative')
            return
        
        ui.notify('Połączono z ESP32!', type='positive')
        
        # Utwórz kontroler
        controller = RobotController(ws_client)
        
        # Kalibruj pozycję startową używając czujników ToF
        ui.notify('Kalibracja pozycji startowej...', type='info')
        calibration_success = controller.calibrate_start_position()
        
        if calibration_success:
            ui.notify('Pozycja skalibrowana pomyślnie!', type='positive')
        else:
            ui.notify('Użyto domyślnej pozycji startowej', type='warning')
        
        # Uzbrój robota
        ui.notify('Uzbrajanie robota...', type='info')
        for _ in range(10):
            await ws_client.send_command({"vx": 0.0, "vy": 0.0, "w": 0.0})
            await asyncio.sleep(0.05)
        controller.arm()
        
        ui.notify('Robot uzbrojony! Rozpoczynanie automatycznej eksploracji...', type='positive')
        
        # Rozpocznij automatyczną eksplorację
        running = True
        asyncio.create_task(update_dashboard())
        
        # Automatycznie rozpocznij eksplorację
        start_exploration()
        
    except Exception as e:
        ui.notify(f'Błąd połączenia: {e}', type='negative')
        print(f"[DASHBOARD] Connection error: {e}")

# ============================================================
# UI LAYOUT
# ============================================================

with ui.row().classes('w-full justify-center items-center bg-gray-900 p-4'):
    ui.label('🤖 Dashboard Rzeczywistego Robota - ESP32-S3').classes('text-h4 text-white')

with ui.row().classes('w-full justify-center gap-4 p-4'):
    # Lewa kolumna - Mapa SLAM
    with ui.card().classes('w-1/2 bg-gray-800'):
        ui.label('🗺️ Mapa SLAM').classes('text-h6 text-white mb-2')
        map_image = ui.image().classes('w-full h-auto border-2 border-gray-600')
        # Inicjalizuj pustym obrazem
        blank_img = np.full((*DISPLAY_SIZE, 3), 38, dtype=np.uint8)
        _, buf = cv2.imencode('.jpg', blank_img)
        blank_b64 = base64.b64encode(buf).decode('utf-8')
        map_image.set_source(f'data:image/jpeg;base64,{blank_b64}')
    
    # Prawa kolumna - Telemetria i Kontrola
    with ui.card().classes('w-1/2 bg-gray-800 p-4'):
        ui.label('📊 Telemetria').classes('text-h6 text-white mb-4')
        
        status_text = ui.label('Status: DISARMED').classes('text-white text-lg mb-2')
        phase_text = ui.label('Faza: IDLE').classes('text-white text-lg mb-2')
        pos_text = ui.label('Pozycja: X=0.0, Y=0.0, Heading=0.0°').classes('text-white text-lg mb-2')
        cells_text = ui.label('Odwiedzone komórki: 0').classes('text-white text-lg mb-2')
        goal_text = ui.label('Cel: Brak').classes('text-white text-lg mb-2')
        
        ui.separator().classes('my-4')
        
        ui.label('🎮 Sterowanie').classes('text-h6 text-white mb-4')
        
        with ui.column().classes('w-full gap-2'):
            ui.button('🔌 Połącz z ESP32', on_click=lambda: asyncio.create_task(connect_to_esp32())).props('color=blue').classes('w-full')
            ui.button('🔓 Uzbrój', on_click=arm_robot).props('color=green').classes('w-full')
            ui.button('🔒 Rozbrój', on_click=disarm_robot).props('color=orange').classes('w-full')
            ui.button('🚀 Start Eksploracji', on_click=start_exploration).props('color=positive').classes('w-full')
            ui.button('🛑 E-STOP', on_click=stop_robot).props('color=negative').classes('w-full')

ui.run(port=DASHBOARD_PORT, title='Real Robot Dashboard', favicon='🤖')
