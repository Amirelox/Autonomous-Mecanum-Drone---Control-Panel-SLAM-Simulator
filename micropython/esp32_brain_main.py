"""
ESP32-S3 "Brain" Client for Mecanum Robot
==========================================
Klient MicroPython dla drugiego ESP32-S3 który działa jako "mózg" robota.
Komunikuje się z pierwszym ESP32 (Hardware Controller) przez UART.

Architektura:
- ESP32 #1 (Hardware): silniki, czujniki, API UART
- ESP32 #2 (Brain): ten plik - algorytmy DFS, SLAM, nawigacja

Wymagane biblioteki MicroPython:
- uasyncio (wbudowane)
- ujson (wbudowane)
- machine (wbudowane)
"""

import uasyncio as asyncio
import ujson as json
import time
import math
from machine import UART

# Parametry fizyczne robota
CELL_SIZE = 30.0       # Rozmiar komórki logicznej [mm]
WALL_THICK = 2.0       # Grubość ściany [mm]
PATH_WIDTH = CELL_SIZE - WALL_THICK  # Szerokość ścieżki [mm]
ROBOT_W_WIDTH = 100.0  # Szerokość robota [mm] (~10 cm)
ROBOT_L_LENGTH = 160.0 # Długość robota [mm] (~16 cm)
SENSOR_RADIUS = ROBOT_L_LENGTH / 2.0  # Promień sensora od środka [mm]
SENSOR_RANGE = 2000.0  # Zasięg sensora [mm] (~2m)

# Konfiguracja sensorów
NUM_SENSORS = 6
RAYS_PER_SENSOR = 15
SENSOR_ANGLES_DEG = [0, 60, 120, 180, 240, 300]

# Watchdog timeout [ms]
WATCHDOG_TIMEOUT_MS = 350

# Częstotliwość wysyłania komend [Hz]
COMMAND_RATE_HZ = 25

# Konfiguracja UART do ESP32 Hardware
UART_PORT = 2
UART_BAUDRATE = 115200
UART_TX_PIN = 17  # GPIO17
UART_RX_PIN = 16  # GPIO16

# ============================================================
# KOMUNIKATOR UART DO ESP32 HARDWARE
# ============================================================

class UARTHardwareClient:
    """
    Klient UART łączący ESP32 Brain z ESP32 Hardware.
    
    Wysyła komendy i odbiera dane przez UART.
    Protokół: JSON linia po linii zakończona \n
    """
    
    def __init__(self):
        self.uart = None
        self.connected = False
        self.last_command_time = 0
    
    def init(self):
        """Inicjalizuje połączenie UART."""
        try:
            self.uart = UART(
                UART_PORT,
                baudrate=UART_BAUDRATE,
                tx=UART_TX_PIN,
                rx=UART_RX_PIN,
                timeout=100  # ms
            )
            self.connected = True
            print("[UART] Connected to Hardware Controller")
            return True
        except Exception as e:
            print(f"[UART] Connection error: {e}")
            return False
    
    def send_command(self, command: dict):
        """Wysyła komendę do ESP32 Hardware."""
        if not self.uart or not self.connected:
            return
        
        try:
            msg = json.dumps(command) + "\n"
            self.uart.write(msg.encode('utf-8'))
            self.last_command_time = time.ticks_ms()
        except Exception as e:
            print(f"[UART] Send error: {e}")
    
    def receive_response(self, timeout_ms: int = 100):
        """
        Odbiera odpowiedź od ESP32 Hardware.
        
        Returns:
            dict: Dane lub None
        """
        if not self.uart or not self.connected:
            return None
        
        try:
            start = time.ticks_ms()
            line = b""
            
            while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
                if self.uart.any():
                    char = self.uart.read(1)
                    if char == b"\n":
                        break
                    line += char
            
            if line:
                data = json.loads(line.decode('utf-8'))
                return data
            
            return None
        except Exception as e:
            print(f"[UART] Receive error: {e}")
            return None
    
    def request_sensors(self):
        """Żąda danych z czujników od ESP32 Hardware."""
        self.send_command({"cmd": "get_sensors"})
        return self.receive_response(timeout_ms=200)
    
    def send_motors(self, fl: float, fr: float, rl: float, rr: float):
        """Wysyła prędkości silników do ESP32 Hardware."""
        self.send_command({
            "cmd": "set_motors",
            "fl": fl,
            "fr": fr,
            "rl": rl,
            "rr": rr
        })


# ============================================================
# PLACEHOLDER: API CZUJNIKÓW (przez UART)
# ============================================================

class SensorAPI:
    """
    Klasa obsługująca czujniki robota przez ESP32 Hardware (UART).
    """
    
    def __init__(self, uart_client: UARTHardwareClient = None):
        self.initialized = False
        self.last_read_time = 0
        self.uart_client = uart_client
        
    def init(self, uart_client: UARTHardwareClient = None):
        """Inicjalizacja połączenia z ESP32 Hardware."""
        if uart_client:
            self.uart_client = uart_client
        self.initialized = True
        print("[SENSORS] Initialized (will receive data via UART)")
    
    def read_lidar(self):
        """
        Odczyt danych z sensorów ToF VL53L7CX (przez UART).
        
        Returns:
            list[dict]: Lista słowników z danymi dla każdego promienia
        """
        if not self.uart_client or not self.uart_client.connected:
            # Placeholder - symulowane dane
            return [{"d": SENSOR_RANGE, "hit": False} for _ in range(NUM_SENSORS * RAYS_PER_SENSOR)]
        
        # Żądaj danych z czujników
        response = self.uart_client.request_sensors()
        
        if response and "lidar" in response:
            return response["lidar"]
        
        # Fallback - placeholder
        return [{"d": SENSOR_RANGE, "hit": False} for _ in range(NUM_SENSORS * RAYS_PER_SENSOR)]
    
    def read_imu(self):
        """
        Odczyt danych z IMU ICM-20948 (przez UART).
        
        Returns:
            dict: Słownik z danymi IMU
        """
        if not self.uart_client or not self.uart_client.connected:
            return {
                "ax": 0.0, "ay": 0.0, "az": 9.81,
                "gx": 0.0, "gy": 0.0, "gz": 0.0,
                "mx": 0.0, "my": 0.0, "mz": 0.0,
                "heading": 0.0
            }
        
        response = self.uart_client.request_sensors()
        
        if response and "imu" in response:
            return response["imu"]
        
        # Placeholder
        return {
            "ax": 0.0, "ay": 0.0, "az": 9.81,
            "gx": 0.0, "gy": 0.0, "gz": 0.0,
            "mx": 0.0, "my": 0.0, "mz": 0.0,
            "heading": 0.0
        }
    
    def read_encoders(self):
        """
        Odczyt enkoderów silników (przez UART).
        
        Returns:
            dict: Słownik z licznikami impulsów
        """
        if not self.uart_client or not self.uart_client.connected:
            return {"fl": 0, "fr": 0, "rl": 0, "rr": 0}
        
        response = self.uart_client.request_sensors()
        
        if response and "encoders" in response:
            return response["encoders"]
        
        # Placeholder
        return {"fl": 0, "fr": 0, "rl": 0, "rr": 0}
    
    def get_position_estimate(self):
        """
        Szacowanie pozycji na podstawie danych od ESP32 Hardware.
        
        Returns:
            tuple: (pos_x, pos_y, heading)
        """
        # TODO: Implementacja fuzji sensorów gdy API będzie gotowe
        # Placeholder
        return (0.0, 0.0, 0.0)


# ============================================================
# KLASY POMOCNICZE
# ============================================================

def clamp(value: float, min_val: float = -1.0, max_val: float = 1.0) -> float:
    """Ogranicza wartość do zakresu [min_val, max_val]."""
    return max(min_val, min(max_val, value))


def mix_mecanum(vx: float, vy: float, w: float) -> list:
    """
    Miksuje wektor ruchu ciała na prędkości 4 kół mecanum.
    
    Równania kinematyczne:
        FL = vy + vx + w
        FR = vy - vx - w
        RL = vy - vx + w
        RR = vy + vx - w
    
    Args:
        vx: Prędkość boczna [-1, 1] (prawo+)
        vy: Prędkość przód/tył [-1, 1] (przód+)
        w: Prędkość obrotowa [-1, 1] (CW+)
    
    Returns:
        list: [FL, FR, RL, RR] przeskalowane do [-1, 1]
    """
    fl = vy + vx + w
    fr = vy - vx - w
    rl = vy - vx + w
    rr = vy + vx - w
    
    # Skalowanie, aby żadne koło nie przekroczyło 1.0
    max_val = max(abs(fl), abs(fr), abs(rl), abs(rr), 1.0)
    if max_val > 1.0:
        fl /= max_val
        fr /= max_val
        rl /= max_val
        rr /= max_val
    
    return [fl, fr, rl, rr]


# ============================================================
# KLASA STEROWNIKA (DFS + SLAM)
# ============================================================

class RobotController:
    """
    Główna klasa sterująca robotem (działa na ESP32 Brain).
    
    Implementuje:
    - Prostą eksplorację DFS
    - Podstawowe SLAM (mapa probabilistyczna)
    - Sterowanie ruchem przez UART
    - Watchdog i bezpieczeństwo
    """
    
    def __init__(self, uart_client: UARTHardwareClient = None):
        self.armed = False
        self.estop = False
        self.finished = False
        
        # Pozycja i orientacja
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.heading = 0.0
        
        # Cel ruchu
        self.target_x = None
        self.target_y = None
        
        # Mapa logiczna ścian (41x41 kafelków, start z (1,1))
        self.logic_map = [[-1] * 41 for _ in range(41)]
        self.logic_map[1][1] = 0  # Startowa komórka wolna
        
        # Mapa i eksploracja DFS
        self.visited_cells = set()
        self.visited_cells.add((1, 1))
        self.path_stack = []
        self.current_cell = (1, 1)
        self.max_r = 1
        self.max_c = 1
        
        # Stan maszyny stanów
        self.exploration_done = False
        self.fast_run = False
        self.goal_cell = None
        self.optimized_path = []
        
        # Watchdog
        self.last_command_time = 0
        
        # Połączenie z ESP32 Hardware
        self.uart_client = uart_client
        
        # Sensory (przez UART)
        self.sensors = SensorAPI(uart_client)
        
        # Telemetria
        self.telemetry = {}
    
    def arm(self):
        """Uzbraja robota (wymaga wysłania komendy neutralnej)."""
        self.armed = True
        print("[CONTROLLER] Armed")
    
    def disarm(self):
        """Rozbraja robota."""
        self.armed = False
        self.estop = False
        print("[CONTROLLER] Disarmed")
    
    def emergency_stop(self):
        """Natychmiastowe zatrzymanie awaryjne."""
        self.estop = True
        self.armed = False
        print("[CONTROLLER] E-STOP ACTIVATED")
    
    def resume(self):
        """Wznowienie po E-stop (następnie wyślij komendę neutralną)."""
        self.estop = False
        print("[CONTROLLER] Resume - send neutral command to arm")
    
    def calibrate_start_position(self):
        """
        Kalibruje pozycję startową robota używając czujników ToF.
        
        Robot znajduje się w kafelku startowym i mierzy odległości do ścian
        aby precyzyjnie określić swoją pozycję (x, y) względem środka kafelka.
        """
        print("[CALIBRATION] Starting position calibration...")
        
        try:
            # Odczytaj dane z czujników
            lidar_data = self.sensors.read_lidar()
            
            # Przybliżone indeksy sensorów dla kierunków
            front_sensor_idx = 0      # 0° - przód
            right_sensor_idx = 2      # ~120° - prawo
            back_sensor_idx = 3       # 180° - tył
            left_sensor_idx = 5       # ~300° (-60°) - lewo
            
            # Pobierz odległości (w mm)
            d_front = lidar_data[front_sensor_idx]["d"] if front_sensor_idx < len(lidar_data) else SENSOR_RANGE
            d_right = lidar_data[right_sensor_idx]["d"] if right_sensor_idx < len(lidar_data) else SENSOR_RANGE
            d_back = lidar_data[back_sensor_idx]["d"] if back_sensor_idx < len(lidar_data) else SENSOR_RANGE
            d_left = lidar_data[left_sensor_idx]["d"] if left_sensor_idx < len(lidar_data) else SENSOR_RANGE
            
            print(f"[CALIBRATION] Distances - Front: {d_front:.1f}, Right: {d_right:.1f}, Back: {d_back:.1f}, Left: {d_left:.1f}")
            
            # Sprawdź czy wszystkie pomiary są sensowne
            max_valid_distance = PATH_WIDTH * 1.5
            
            if any(d > max_valid_distance for d in [d_front, d_right, d_back, d_left]):
                print("[CALIBRATION] WARNING: Some sensors don't detect walls. Using default position.")
                self.pos_x = WALL_THICK + PATH_WIDTH / 2
                self.pos_y = WALL_THICK + PATH_WIDTH / 2
                self.heading = 0.0
                return False
            
            # Oblicz pozycję X (od lewej ściany)
            calculated_x = d_left + ROBOT_W_WIDTH / 2
            
            # Oblicz pozycję Y (od tylnej ściany)
            calculated_y = d_back + ROBOT_L_LENGTH / 2
            
            # Walidacja: czy pozycja jest wewnątrz kafelka
            if 0 <= calculated_x <= PATH_WIDTH and 0 <= calculated_y <= PATH_WIDTH:
                self.pos_x = calculated_x
                self.pos_y = calculated_y
                self.heading = 0.0
                
                print(f"[CALIBRATION] ✓ Position calibrated: X={self.pos_x:.2f}, Y={self.pos_y:.2f}, Heading=0°")
                print(f"[CALIBRATION] Offset from center: dx={calculated_x - PATH_WIDTH/2:.2f}, dy={calculated_y - PATH_WIDTH/2:.2f}")
                
                return True
            else:
                print(f"[CALIBRATION] ✗ Calculated position out of bounds! Using default.")
                self.pos_x = WALL_THICK + PATH_WIDTH / 2
                self.pos_y = WALL_THICK + PATH_WIDTH / 2
                self.heading = 0.0
                return False
                
        except Exception as e:
            print(f"[CALIBRATION] Error during calibration: {e}")
            self.pos_x = WALL_THICK + PATH_WIDTH / 2
            self.pos_y = WALL_THICK + PATH_WIDTH / 2
            self.heading = 0.0
            return False
    
    def update_sensors(self):
        """Aktualizuje dane z czujników przez UART i aktualizuje mapę logiczną."""
        if not self.sensors.initialized:
            self.sensors.init()
        
        # Odczytaj dane z ESP32 Hardware
        lidar_data = self.sensors.read_lidar()
        imu_data = self.sensors.read_imu()
        encoders = self.sensors.read_encoders()
        
        # Aktualizuj pozycję z danych sensorów
        pos = self.sensors.get_position_estimate()
        new_x, new_y, new_heading = pos
        
        # Sprawdź czy pozycja się zmieniła (unikaj dryfu)
        if abs(new_x - self.pos_x) > 0.1 or abs(new_y - self.pos_y) > 0.1:
            self.pos_x = new_x
            self.pos_y = new_y
            self.heading = new_heading
            
            # Aktualizuj aktualną komórkę logiczną na podstawie pozycji
            cell_r = int(self.pos_y / CELL_SIZE)
            cell_c = int(self.pos_x / CELL_SIZE)
            
            # Sprawdź czy w granicach mapy
            if 0 <= cell_r < 41 and 0 <= cell_c < 41:
                self.current_cell = (cell_r, cell_c)
                
                # Oznacz jako odwiedzoną
                if self.current_cell not in self.visited_cells:
                    self.visited_cells.add(self.current_cell)
                    self.logic_map[cell_r][cell_c] = 0  # Wolna przestrzeń
                
                # Aktualizuj max wymiary
                if cell_r > self.max_r:
                    self.max_r = cell_r
                if cell_c > self.max_c:
                    self.max_c = cell_c
        
        return lidar_data, imu_data, encoders
    
    def find_next_target(self):
        """
        Znajduje następną komórkę do odwiedzenia w eksploracji DFS.
        Sprawdza mapę logiczną ścian przed podjęciem decyzji.
        """
        r, c = self.current_cell
        # Krok o 2 dla kafelków (pomijamy środek przejścia)
        directions = [(-2, 0), (0, 2), (2, 0), (0, -2)]  # N, E, S, W
        
        for dr, dc in directions:
            nr, nc = r + dr, c + dc
            
            # Sprawdź czy w granicach mapy
            if not (0 <= nr < 41 and 0 <= nc < 41):
                continue
            
            # Sprawdź czy przejście (środek) jest wolne od ścian
            mid_r, mid_c = r + dr//2, c + dc//2
            mid_free = self.logic_map[mid_r][mid_c] <= 0
            
            # Sprawdź czy cel jest wolny od ścian
            target_free = self.logic_map[nr][nc] <= 0
            
            # Sprawdź czy nieodwiedzona
            not_visited = (nr, nc) not in self.visited_cells
            
            if mid_free and target_free and not_visited:
                return (nr, nc)
        
        # Brak nieodwiedzonych sąsiadów - cofnij się
        if self.path_stack:
            return self.path_stack.pop()
        
        return None
    
    def compute_shortest_path(self, start, goal):
        """
        Oblicza najkrótszą ścieżkę BFS od start do goal.
        Sprawdza mapę logiczną ścian przed podjęciem decyzji.
        """
        queue = [(start, [start])]
        visited = {start}
        head = 0
        
        while head < len(queue):
            (r, c), path = queue[head]
            head += 1
            
            if (r, c) == goal:
                return path
            
            # Krok o 2 dla kafelków
            for dr, dc in [(-2, 0), (0, 2), (2, 0), (0, -2)]:
                nr, nc = r + dr, c + dc
                
                # Sprawdź czy w granicach mapy
                if not (0 <= nr < 41 and 0 <= nc < 41):
                    continue
                
                # Sprawdź czy przejście (środek) jest wolne
                mid_r, mid_c = r + dr//2, c + dc//2
                mid_free = self.logic_map[mid_r][mid_c] == 0
                
                # Sprawdź czy cel jest wolny
                target_free = self.logic_map[nr][nc] == 0
                
                if mid_free and target_free and (nr, nc) not in visited:
                    visited.add((nr, nc))
                    queue.append(((nr, nc), path + [(nr, nc)]))
        
        return []  # Brak ścieżki
    
    def update_exploration_state(self):
        """
        Aktualizuje stan eksploracji i sprawdza czy zakończyć fazę DFS.
        """
        if self.exploration_done or self.fast_run:
            return False
        
        # Kryterium stopu: Pusty stos, powrót na start i zbadanie więcej niż 1 komórki
        if len(self.path_stack) == 0 and self.current_cell == (1, 1) and len(self.visited_cells) > 1:
            self.exploration_done = True
            print("[EXPLORATION] DFS completed!")
            
            # Oblicz cel (narożnik)
            max_r = max(r for r, c in self.visited_cells)
            max_c = max(c for r, c in self.visited_cells)
            
            self.goal_cell = (max_r, max_c)
            print(f"[GOAL] Target cell: {self.goal_cell}")
            
            # Oblicz optymalną ścieżkę BFS
            self.optimized_path = self.compute_shortest_path((1, 1), self.goal_cell)
            if self.optimized_path:
                self.optimized_path = self.optimized_path[1:]  # Pomiń start
            
            self.fast_run = True
            print(f"[FAST RUN] Path length: {len(self.optimized_path)} cells")
            return True
        
        return False
    
    def explore_next(self):
        """
        Eksploracja DFS - znajduje następną komórkę do odwiedzenia.
        """
        # Sprawdź czy zakończyć eksplorację
        if self.update_exploration_state():
            return None
        
        # Jeśli już w fast_run, użyj nawigacji po ścieżce
        if self.fast_run:
            return None
        
        # Znajdź następną komórkę
        next_cell = self.find_next_target()
        
        if next_cell is None:
            return (0.0, 0.0, 0.0)
        
        # Dodaj do stosu jeśli to nowa komórka
        if next_cell not in self.visited_cells:
            self.path_stack.append(self.current_cell)
            self.visited_cells.add(next_cell)
            self.current_cell = next_cell
            
            # Aktualizuj max wymiary
            r, c = next_cell
            if r > self.max_r:
                self.max_r = r
            if c > self.max_c:
                self.max_c = c
        
        # Konwersja komórki logicznej na współrzędne fizyczne
        target_x = next_cell[1] * CELL_SIZE
        target_y = next_cell[0] * CELL_SIZE
        
        # Prosty regulator do celu
        dx = target_x - self.pos_x
        dy = target_y - self.pos_y
        dist = math.sqrt(dx**2 + dy**2)
        
        if dist < 5.0:
            return (0.0, 0.0, 0.0)
        
        speed = 0.3
        angle_to_target = math.atan2(dy, dx)
        angle_error = angle_to_target - self.heading
        
        # Normalizacja kąta
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
        
        vx = speed * math.cos(angle_to_target)
        vy = speed * math.sin(angle_to_target)
        w = clamp(angle_error * 0.5, -0.5, 0.5)
        
        return (vx, vy, w)
    
    def get_movement_command(self) -> dict:
        """
        Generuje komendę ruchu.
        """
        if self.estop or not self.armed:
            return {"vx": 0.0, "vy": 0.0, "w": 0.0}
        
        # FAZA 2: FAST RUN - nawigacja po optymalnej ścieżce BFS
        if self.fast_run:
            if self.optimized_path:
                # Pobierz następną komórkę ze ścieżki
                next_cell = self.optimized_path[0]
                target_x = next_cell[1] * CELL_SIZE
                target_y = next_cell[0] * CELL_SIZE
                
                dx = target_x - self.pos_x
                dy = target_y - self.pos_y
                dist = math.sqrt(dx**2 + dy**2)
                
                # Sprawdź czy dotarliśmy do komórki
                if dist < CELL_SIZE / 2:
                    self.optimized_path.pop(0)
                    
                    # Sprawdź czy to cel końcowy
                    if next_cell == self.goal_cell:
                        print("[FAST RUN] Goal reached!")
                        self.finished = True
                        return {"vx": 0.0, "vy": 0.0, "w": 0.0}
                    
                    if not self.optimized_path:
                        self.finished = True
                        return {"vx": 0.0, "vy": 0.0, "w": 0.0}
                    
                    next_cell = self.optimized_path[0]
                    target_x = next_cell[1] * CELL_SIZE
                    target_y = next_cell[0] * CELL_SIZE
                    dx = target_x - self.pos_x
                    dy = target_y - self.pos_y
                
                # Nawigacja do następnej komórki (szybsza prędkość)
                speed = 0.6
                angle_to_target = math.atan2(dy, dx)
                angle_error = angle_to_target - self.heading
                
                while angle_error > math.pi:
                    angle_error -= 2 * math.pi
                while angle_error < -math.pi:
                    angle_error += 2 * math.pi
                
                vx = speed * math.cos(angle_to_target)
                vy = speed * math.sin(angle_to_target)
                w = clamp(angle_error * 0.5, -0.5, 0.5)
                
                return {"vx": vx, "vy": vy, "w": w}
            else:
                self.finished = True
                return {"vx": 0.0, "vy": 0.0, "w": 0.0}
        
        # FAZA 1: EKSPLORACJA DFS
        if self.target_x is not None and self.target_y is not None:
            dx = self.target_x - self.pos_x
            dy = self.target_y - self.pos_y
            dist = math.sqrt(dx**2 + dy**2)
            
            if dist < 5.0:
                self.target_x = None
                self.target_y = None
                return {"vx": 0.0, "vy": 0.0, "w": 0.0}
            
            speed = 0.5
            angle_to_target = math.atan2(dy, dx)
            angle_error = angle_to_target - self.heading
            
            while angle_error > math.pi:
                angle_error -= 2 * math.pi
            while angle_error < -math.pi:
                angle_error += 2 * math.pi
            
            vx = speed * math.cos(angle_to_target)
            vy = speed * math.sin(angle_to_target)
            w = clamp(angle_error * 0.5, -0.5, 0.5)
            
            return {"vx": vx, "vy": vy, "w": w}
        else:
            result = self.explore_next()
            if result:
                vx, vy, w = result
                return {"vx": vx, "vy": vy, "w": w}
            else:
                return {"vx": 0.0, "vy": 0.0, "w": 0.0}
    
    def run_exploration_step(self):
        """
        Wykonuje jeden krok eksploracji/nawigacji i wysyła komendę do ESP32 Hardware.
        """
        if self.estop or not self.armed:
            if self.uart_client and self.uart_client.connected:
                self.uart_client.send_motors(0.0, 0.0, 0.0, 0.0)
            return
        
        # Aktualizuj sensory
        self.update_sensors()
        
        # Pobierz komendę ruchu
        command = self.get_movement_command()
        
        # Konwertuj vx, vy, w na prędkości kół
        wheel_speeds = mix_mecanum(command["vx"], command["vy"], command["w"])
        
        # Wyślij do ESP32 Hardware
        if self.uart_client and self.uart_client.connected:
            self.uart_client.send_motors(wheel_speeds[0], wheel_speeds[1], wheel_speeds[2], wheel_speeds[3])
        
        # Aktualizuj watchdog
        self.last_command_time = time.ticks_ms()
    
    def check_watchdog(self, current_time_ms: int) -> bool:
        """
        Sprawdza watchdog.
        """
        elapsed = time.ticks_diff(current_time_ms, self.last_command_time)
        if elapsed > WATCHDOG_TIMEOUT_MS:
            print(f"[WATCHDOG] Timeout! ({elapsed}ms > {WATCHDOG_TIMEOUT_MS}ms)")
            self.disarm()
            return False
        return True


# ============================================================
# FUNKCJA GŁÓWNA
# ============================================================

async def main():
    """Główna funkcja programu."""
    print("=" * 60)
    print("ESP32 Brain Client for Mecanum Robot")
    print("=" * 60)
    
    # Utwórz klienta UART do ESP32 Hardware
    uart_client = UARTHardwareClient()
    
    # Inicjalizuj UART
    if not uart_client.init():
        print("[ERROR] Nie można zainicjalizować UART")
        return
    
    # Utwórz kontroler robota
    controller = RobotController(uart_client)
    
    # Kalibruj pozycję startową
    print("[MAIN] Calibrating start position...")
    calibration_success = controller.calibrate_start_position()
    
    if calibration_success:
        print("[MAIN] ✓ Position calibrated")
    else:
        print("[MAIN] ⚠ Using default position")
    
    # Uzbrój robota (wyślij komendy neutralne)
    print("[MAIN] Arming robot...")
    for _ in range(10):
        uart_client.send_motors(0.0, 0.0, 0.0, 0.0)
        await asyncio.sleep_ms(50)
    controller.arm()
    
    print("[MAIN] Starting exploration...")
    
    # Główna pętla
    try:
        while not controller.finished:
            # Sprawdź watchdog
            if not controller.check_watchdog(time.ticks_ms()):
                # Rozbrojony - wyślij komendę neutralną aby uzbroić
                uart_client.send_motors(0.0, 0.0, 0.0, 0.0)
                await asyncio.sleep_ms(500)
                controller.arm()
                continue
            
            # Wykonaj krok eksploracji
            controller.run_exploration_step()
            
            # Czekaj przed następną iteracją
            await asyncio.sleep_ms(int(1000 / COMMAND_RATE_HZ))
            
    except KeyboardInterrupt:
        print("\n[EXIT] Program terminated by user")
    finally:
        # Zatrzymaj robota
        uart_client.send_motors(0.0, 0.0, 0.0, 0.0)
        print("[MAIN] Stopped")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[EXIT] Program terminated by user")
    except Exception as e:
        print(f"\n[ERROR] Unexpected error: {e}")
        import sys
        sys.print_exception(e)
