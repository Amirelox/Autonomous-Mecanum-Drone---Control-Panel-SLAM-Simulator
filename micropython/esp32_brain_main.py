"""
ESP32-S3 "Brain" Client for Mecanum Robot - V1.2 Production
==========================================================
Natywny system operacyjny dla ESP32 #2 (Mózg). 
W pełni zintegrowany z dedykowanym API sensorów ToF, mikserem bazy,
ślepy DFS/BFS oraz wbudowanym serwerem Web HMI dla telefonu.
"""

import uasyncio as asyncio
import ujson as json
import time
import math
from machine import UART, Pin
import network
import usocket as socket

# ============================================================
# 📐 PARAMETRY FIZYCZNE (Skala Turniejowa - mm)
# ============================================================
CELL_SIZE = 300.0       # Rozmiar komórki labiryntu: 300 mm (30 cm)
WALL_THICK = 12.0       # Grubość ściany: 12 mm
PATH_WIDTH = CELL_SIZE - WALL_THICK  # Szerokość ścieżki: 288 mm
ROBOT_W_WIDTH = 100.0  # Szerokość robota: 100 mm (10 cm)
ROBOT_L_LENGTH = 160.0 # Długość robota: 160 mm (16 cm)
SENSOR_RADIUS = ROBOT_L_LENGTH / 2.0  
SENSOR_RANGE = 2000.0  # Maksymalny zasięg czujników ToF: 2000 mm (2 m)

NUM_SENSORS = 6
RAYS_PER_SENSOR = 15
SENSOR_ANGLES_DEG = [0, 60, 120, 180, 240, 300]

WATCHDOG_TIMEOUT_MS = 350
COMMAND_RATE_HZ = 25

UART_PORT = 2
UART_BAUDRATE = 115200
UART_TX_PIN = 17  
UART_RX_PIN = 16  

# ============================================================
# 🔌 TRANSMISJA UART DO ESP32 HARDWARE
# ============================================================
class UARTHardwareClient:
    def __init__(self):
        self.uart = None
        self.connected = False
    
    def init(self):
        try:
            self.uart = UART(UART_PORT, baudrate=UART_BAUDRATE, tx=UART_TX_PIN, rx=UART_RX_PIN, timeout=30)
            self.connected = True
            print("[UART] Połączono z Hardware Controller (ESP32 #1)")
            return True
        except Exception as e:
            print("[UART] Błąd magistrali przewodowej:", e)
            return False
    
    def send_command(self, command: dict):
        if not self.uart or not self.connected: return
        try:
            self.uart.write((json.dumps(command) + "\n").encode('utf-8'))
        except:
            pass
    
    def read_raw_telemetry(self):
        """Pobiera surową linię tekstu z bufora UART."""
        if not self.uart or not self.connected: return None
        try:
            line = self.uart.readline()
            if line: return json.loads(line.decode('utf-8'))
        except:
            pass
        return None

    def send_motors(self, fl: float, fr: float, rl: float, rr: float):
        self.send_command({"raw": [fl, fr, rl, rr]})

# ============================================================
# 🎯 INTEGRACJA: PASYWNE API CZUJNIKÓW (PARSER TELEMETRII)
# ============================================================
class SensorAPI:
    def __init__(self, uart_client: UARTHardwareClient):
        self.uart_client = uart_client
        
    def get_processed_lidar(self) -> list:
        """
        Odpytuje ESP32 Hardware i przetwarza surowe odczyty ToF
        na ujednoliconą, bezpieczną strukturę logiczną drona.
        """
        self.uart_client.send_command({"cmd": "get_sensors"})
        raw_data = self.uart_client.read_raw_telemetry()
        
        # Fallback bezpieczeństwa w przypadku zerwania ramki lub przewodu UART
        if not raw_data or "lidar" not in raw_data:
            return [{"d": SENSOR_RANGE, "hit": False} for _ in range(NUM_SENSORS)]
        
        processed_beams = []
        for sensor in raw_data["lidar"]:
            dist = float(sensor.get("d", SENSOR_RANGE))
            # Sensor zgłasza trafienie (hit=True) tylko gdy przeszkoda jest w zasięgu operacyjnym
            processed_beams.append({
                "d": dist,
                "hit": dist < (SENSOR_RANGE - 50.0)
            })
        return processed_beams

# ============================================================
# 🏎️ KINEMATYKA KÓŁ MECANUM (Zgodna z Twoim API 2.3)
# ============================================================
def mix_mecanum(vx: float, vy: float, w: float) -> list:
    fl = vy + vx + w
    fr = vy - vx - w
    rl = vy - vx + w
    rr = vy + vx - w
    
    max_val = max(abs(fl), abs(fr), abs(rl), abs(rr), 1.0)
    if max_val > 1.0:
        fl /= max_val; fr /= max_val; rl /= max_val; rr /= max_val
    return [fl, fr, rl, rr]

# ============================================================
# 🤖 RDZEŃ AUTONOMII (DFS + BFS + Korekta Odometrii)
# ============================================================
class RobotController:
    def __init__(self, sensor_api: SensorAPI):
        self.armed = False
        self.estop = False
        self.finished = False
        
        # Start ze środka kafelka (1,1) w mm
        self.pos_x = WALL_THICK + PATH_WIDTH / 2.0
        self.pos_y = WALL_THICK + PATH_WIDTH / 2.0
        self.heading = 0.0
        
        self.logic_map = [[-1] * 41 for _ in range(41)]
        self.logic_map[1][1] = 0  
        
        self.visited_cells = set()
        self.visited_cells.add((1, 1))
        self.path_stack = []
        self.current_cell = (1, 1)
        self.max_r = 1
        self.max_c = 1
        
        self.exploration_done = False
        self.fast_run = False
        self.goal_cell = None
        self.optimized_path = []
        
        self.sensors = sensor_api
        self.last_cmd_sent = {"vx": 0.0, "vy": 0.0, "w": 0.0}

    def calibrate_start_position(self, lidar_data):
        """Autokalibracja startowa ToF ze ścianami (Twój pomysł)."""
        try:
            d_back = lidar_data[3]["d"]
            d_left = lidar_data[4]["d"] * 0.866 # Rzut cosinusowy czujnika skośnego 240°
            
            self.pos_x = d_left + (ROBOT_W_WIDTH / 2.0)
            self.pos_y = d_back + (ROBOT_L_LENGTH / 2.0)
            self.heading = 0.0
            print(f"[CALIBRATION] ✓ Pozycja zsynchronizowana: X={self.pos_x:.1f}mm, Y={self.pos_y:.1f}mm")
            return True
        except:
            return False

    def update_slam(self, lidar_data):
        """Aktualizuje pozycję na bazie odometrii poleceń i nanosi ściany z API czujników."""
        dt = 0.04 # 25 Hz
        speed_scale = 180.0 # mm/s przy pełnej mocy
        
        self.heading += self.last_cmd_sent["w"] * 1.2 * dt
        self.pos_x += (self.last_cmd_sent["vx"] * math.cos(self.heading) - self.last_cmd_sent["vy"] * math.sin(self.heading)) * speed_scale * dt
        self.pos_y += (self.last_cmd_sent["vx"] * math.sin(self.heading) + self.last_cmd_sent["vy"] * math.cos(self.heading)) * speed_scale * dt

        cell_c = int(self.pos_x / CELL_SIZE)
        cell_r = int(self.pos_y / CELL_SIZE)
        if 0 <= cell_r < 41 and 0 <= cell_c < 41:
            self.current_cell = (cell_r, cell_c)
            if self.current_cell not in self.visited_cells:
                self.visited_cells.add(self.current_cell)
                self.logic_map[cell_r][cell_c] = 0
            self.max_r = max(self.max_r, cell_r)
            self.max_c = max(self.max_c, cell_c)

        # Korzystanie z ujednoliconej struktury przetworzonej przez SensorAPI
        r, c = self.current_cell
        if lidar_data[0]["hit"] and lidar_data[0]["d"] < 140.0: 
            self.logic_map[r+1][c] = 1 # Wykryto fizyczną ścianę z przodu (Północ)
        else:
            self.logic_map[r+1][c] = 0

    def find_next_target(self):
        r, c = self.current_cell
        for dr, dc in [(-2, 0), (0, 2), (2, 0), (0, -2)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < 41 and 0 <= nc < 41:
                if self.logic_map[r + dr//2][c + dc//2] <= 0 and self.logic_map[nr][nc] <= 0 and (nr, nc) not in self.visited_cells:
                    return (nr, nc)
        return self.path_stack.pop() if self.path_stack else None

    def compute_shortest_path(self, start, goal):
        queue = [(start, [start])]
        visited = {start}
        head = 0
        while head < len(queue):
            (r, c), path = queue[head]; head += 1
            if (r, c) == goal: return path
            for dr, dc in [(-2, 0), (0, 2), (2, 0), (0, -2)]:
                nr, nc = r + dr, c + dc
                if 0 <= nr < 41 and 0 <= nc < 41:
                    if self.logic_map[r+dr//2][c+dc//2] == 0 and self.logic_map[nr][nc] == 0 and (nr, nc) not in visited:
                        visited.add((nr, nc))
                        queue.append(((nr, nc), path + [(nr, nc)]))
        return []

    def get_movement_command(self):
        if self.estop or not self.armed: return {"vx": 0.0, "vy": 0.0, "w": 0.0}
        
        if self.fast_run:
            if self.optimized_path:
                next_cell = self.optimized_path[0]
                tx, ty = next_cell[1] * CELL_SIZE + CELL_SIZE/2, next_cell[0] * CELL_SIZE + CELL_SIZE/2
                dx, dy = tx - self.pos_x, ty - self.pos_y
                if math.sqrt(dx**2 + dy**2) < 15.0:
                    self.optimized_path.pop(0)
                    if next_cell == self.goal_cell:
                        self.finished = True
                        return {"vx": 0.0, "vy": 0.0, "w": 0.0}
                ang = math.atan2(dy, dx) - self.heading
                return {"vx": 0.55 * math.cos(ang), "vy": 0.55 * math.sin(ang), "w": max(-0.25, min(0.25, ang * 0.4))}
            self.finished = True
            return {"vx": 0.0, "vy": 0.0, "w": 0.0}

        if len(self.path_stack) == 0 and self.current_cell == (1, 1) and len(self.visited_cells) > 1:
            self.exploration_done = True
            self.goal_cell = (self.max_r, self.max_c)
            self.optimized_path = self.compute_shortest_path((1, 1), self.goal_cell)[1:]
            self.fast_run = True
            return {"vx": 0.0, "vy": 0.0, "w": 0.0}

        next_cell = self.find_next_target()
        if next_cell is None: return {"vx": 0.0, "vy": 0.0, "w": 0.0}
        
        if next_cell not in self.visited_cells:
            self.path_stack.append(self.current_cell)
            self.visited_cells.add(next_cell)
            self.current_cell = next_cell

        tx, ty = next_cell[1] * CELL_SIZE + CELL_SIZE/2, next_cell[0] * CELL_SIZE + CELL_SIZE/2
        ang = math.atan2(ty - self.pos_y, tx - self.pos_x) - self.heading
        return {"vx": 0.3 * math.cos(ang), "vy": 0.3 * math.sin(ang), "w": max(-0.2, min(0.2, ang * 0.4))}

# ============================================================
# 💻 EMBEDDED WEB DASHBOARD (Natywny Serwer dla Telefonu)
# ============================================================
html_dashboard = """<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Drone Console</title><style>
body { background:#0a0a0a; color:#fff; font-family:sans-serif; text-align:center; padding:20px; }
.btn { width:80%; max-width:300px; padding:15px; margin:10px; font-weight:bold; border:none; border-radius:4px; font-size:16px; cursor:pointer; }
.btn-start { background:#00bc8c; color:#fff; }
.btn-kill { background:#e74c3c; color:#fff; }
.btn-resume { background:#3498db; color:#fff; }
.status-box { background:#111; padding:15px; border-radius:6px; border:1px solid #222; margin:20px auto; max-width:400px; }
h3 { color:#00f0ff; }
</style></head><body>
<h3>🛸 MECANUM DRONE CONSOLE</h3>
<div class="status-box"><div id="mode">Status: OCZEKIWANIE</div><div id="cell">Pozycja: ---</div></div>
<button class="btn btn-start" onclick="send('start')">🚀 START EXPLORATION</button>
<button class="btn btn-kill" onclick="send('kill')">🛑 EMERGENCY KILL</button>
<button class="btn btn-resume" onclick="send('resume')">🔄 RESUME UNLOCK</button>
<script>
function send(cmd) { fetch('/api?cmd=' + cmd); }
setInterval(async () => {
    try {
        let res = await fetch('/api?cmd=status');
        let data = await res.json();
        document.getElementById('mode').innerText = "Tryb: " + data.mode.toUpperCase();
        document.getElementById('cell').innerText = "Siatka DFS: [" + data.cell[0] + ", " + data.cell[1] + "]";
    } catch(e) {}
}, 400);
</script></body></html>"""

hmi_command_signal = None

async def hmi_web_server():
    global hmi_command_signal
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setblocking(False)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('0.0.0.0', 8080))
    s.listen(2)
    
    while True:
        try:
            res = s.accept()
            if res:
                conn, addr = res
                conn.setblocking(True)
                req = conn.recv(512).decode('utf-8')
                
                if "GET /api" in req:
                    if "cmd=start" in req: hmi_command_signal = "start"
                    elif "cmd=kill" in req: hmi_command_signal = "kill"
                    elif "cmd=resume" in req: hmi_command_signal = "resume"
                    
                    from __main__ import current_live_mode, current_live_cell
                    status_payload = {"mode": current_live_mode, "cell": current_live_cell}
                    conn.send("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\r\n" + json.dumps(status_payload))
                else:
                    conn.send("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n" + html_dashboard)
                conn.close()
        except:
            pass
        await asyncio.sleep_ms(40)

# Variables for live synchronization with web display
current_live_mode = "stopped"
current_live_cell = [1, 1]

async def main():
    global hmi_command_signal, current_live_mode, current_live_cell
    
    # 1. Konfiguracja punktu dostępowego bazy (Wi-Fi AP)
    ap = network.WLAN(network.AP_IF)
    ap.active(True)
    ap.config(essid="Mecanum_Drone_AP", password="robot_password_2026")
    print("[WIFI] Połącz się z siecią 'Mecanum_Drone_AP' i otwórz w przeglądarce 192.168.4.1:8080")

    # 2. Uruchomienie asynchronicznej obsługi Web HMI
    asyncio.create_task(hmi_web_server())

    # 3. Synchronizacja połączenia magistrali UART
    uart_client = UARTHardwareClient()
    if not uart_client.init(): return
        
    # Inicjalizacja instancji interfejsu API sensorów oraz głównego kontrolera
    sensor_api = SensorAPI(uart_client)
    controller = RobotController(sensor_api)
    
    # Blokada pętli: Oczekiwanie na kliknięcie przycisku przez operatora HMI
    while True:
        if hmi_command_signal == "start":
            hmi_command_signal = None
            break
        await asyncio.sleep_ms(100)

    # 4. Inicjalne próbkowanie i kalibracja startowa ToF za pomocą SensorAPI
    initial_lidar_profile = sensor_api.get_processed_lidar()
    controller.calibrate_start_position(initial_lidar_profile)
    
    # 5. Bezpieczne uzbrojenie bazy (Neutralne impulsy PWM)
    for _ in range(10):
        uart_client.send_motors(0.0, 0.0, 0.0, 0.0)
        await asyncio.sleep_ms(20)
    controller.arm()
    
    print("[MAIN] Pętla 25 Hz uruchomiona pomyślnie.")
    
    # 6. Główna pętla wykonawcza czasu rzeczywistego
    while not controller.finished:
        if hmi_command_signal:
            if hmi_command_signal == "kill": controller.emergency_stop()
            elif hmi_command_signal == "resume": controller.resume()
            hmi_command_signal = None

        # Pobranie ujednoliconych danych z czujników ToF poprzez instancję API
        lidar_data = sensor_api.get_processed_lidar()
        
        # Przetworzenie danych ToF na mapę logiczną korytarzy labiryntu i aktualizacja SLAM
        controller.update_slam(lidar_data)
        
        # Odświeżenie danych telemetrycznych HMI dla telefonu komórkowego
        current_live_mode = "explore" if not controller.fast_run else "speedrun"
        if controller.estop: current_live_mode = "EMERGENCY STOP"
        current_live_cell = list(controller.current_cell)

        # Obliczenie kolejnego kroku DFS/BFS oraz dystrybucja wektorów kół Mecanum
        command = controller.get_movement_command()
        controller.last_cmd_sent = command
        speeds = mix_mecanum(command["vx"], command["vy"], command["w"])
        
        # Przesłanie ramki wykonawczej po kablu UART do sterownika silników (ESP32 #1)
        uart_client.send_motors(speeds[0], speeds[1], speeds[2], speeds[3])
        
        await asyncio.sleep_ms(int(1000 / COMMAND_RATE_HZ))

    print("[FINISHED] Wyścig zakończony. Odcięcie zasilania silników.")
    uart_client.send_motors(0.0, 0.0, 0.0, 0.0)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except Exception as e:
        print("[CRASH] Fatalny błąd jądra systemu MicroPython:", e)