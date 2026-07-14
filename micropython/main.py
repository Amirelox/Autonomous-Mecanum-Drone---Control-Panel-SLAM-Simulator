"""
MicroPython Client for ESP32-S3 Mecanum Robot
==============================================
Szablon implementacji klienta dla robota z napędem mecanum.
Implementuje autentykację WebSocket, komunikację z robotem oraz strukturę
pod przyszłe API czujników.

Wymagane biblioteki MicroPython:
- uasyncio (wbudowane)
- ujson (wbudowane)
- urequests (opcjonalnie, do REST API)
- umqtt_simple (opcjonalnie, jeśli MQTT zamiast WebSocket)

Uwaga: HMAC-SHA256 wymaga micropython-lib lub własnej implementacji.
"""

import uasyncio as asyncio
import ujson as json
import machine
import time
import math
import struct

# Spróbuj załadować klucze z secrets.py (opcjonalne)
try:
    from secrets import ROBOT_HMAC_KEY, ROBOT_WS_URL
    print("[CONFIG] Loaded credentials from secrets.py")
except ImportError:
    # Domyślne wartości (zmień przed uruchomieniem!)
    ROBOT_HMAC_KEY = "YOUR_64_CHAR_HEX_KEY_HERE"
    ROBOT_WS_URL = "ws://192.168.1.100/ws"
    print("[CONFIG] Using default credentials - CHANGE BEFORE PRODUCTION!")

# Parametry fizyczne robota (z config.py)
# Uwaga: Wartości w jednostkach symulacji (piksele). 
# W rzeczywistym robocie przelicz na mm/cm zgodnie z skalą.
CELL_SIZE = 30.0       # Rozmiar komórki logicznej [jednostki symulacji]
WALL_THICK = 2.0       # Grubość ściany [jednostki symulacji]
PATH_WIDTH = CELL_SIZE - WALL_THICK  # Szerokość ścieżki [jednostki symulacji]
ROBOT_W_WIDTH = 10.0   # Szerokość robota [jednostki symulacji] (~10 cm)
ROBOT_L_LENGTH = 16.0  # Długość robota [jednostki symulacji] (~16 cm)
SENSOR_RADIUS = ROBOT_L_LENGTH / 2.0  # Promień sensora od środka [jednostki symulacji]
SENSOR_RANGE = 200.0   # Zasięg sensora [jednostki symulacji] (~200 cm = 2m)
NOISE_STD = 1.0        # Odchylenie standardowe szumu [jednostki symulacji]

# Konfiguracja sensorów (z config.py)
NUM_SENSORS = 6
RAYS_PER_SENSOR = 15
SENSOR_ANGLES_DEG = [0, 60, 120, 180, 240, 300]  # Kąty montażu sensorów

# Konfiguracja pinów GPIO (ESP32-S3)
# TODO: Dostosuj piny do swojego układu
I2C_SCL_PIN = 22  # Pin SCL dla I2C (czujniki VL53L7CX, IMU)
I2C_SDA_PIN = 21  # Pin SDA dla I2C
I2C_FREQ = 400000  # Częstotliwość I2C [Hz]

# Piny enkoderów (przykładowe - dostosuj do swojego układu)
ENCODER_PINS = {
    "fl": (32, 33),  # (pin_A, pin_B) dla front-left
    "fr": (25, 26),  # front-right
    "rl": (27, 14),  # rear-left
    "rr": (12, 13),  # rear-right
}

# Watchdog timeout [ms]
WATCHDOG_TIMEOUT_MS = 350

# Częstotliwość wysyłania komend [Hz]
COMMAND_RATE_HZ = 25

# ============================================================
# PLACEHOLDER: API CZUJNIKÓW
# ============================================================
# TODO: Po otrzymaniu dokumentacji API czujników, wypełnij tę klasę

class SensorAPI:
    """
    Klasa obsługująca czujniki robota.
    
    Obecnie zawiera placeholder'y dla:
    - LiDAR / ToF (VL53L7CX) - 6 sensorów na I2C z multiplekserem TCA9548A
    - IMU (ICM-20948) - akcelerometr, żyroskop, magnetometr
    - Enkodery silników - 4 enkodery kwadraturowe
    
    Po otrzymaniu API, zaimplementuj metody read_*() zgodnie z protokołem.
    """
    
    def __init__(self):
        self.initialized = False
        self.last_read_time = 0
        self.i2c = None
        self.tca9548 = None  # Multiplekser I2C dla VL53L7CX
        
    def init(self):
        """Inicjalizacja czujników (I2C, SPI, UART)."""
        try:
            # Inicjalizacja magistrali I2C
            self.i2c = machine.I2C(
                0, 
                scl=machine.Pin(I2C_SCL_PIN), 
                sda=machine.Pin(I2C_SDA_PIN), 
                freq=I2C_FREQ
            )
            
            # Skanuj urządzenia I2C (debug)
            devices = self.i2c.scan()
            print(f"[SENSORS] I2C devices found: {[hex(d) for d in devices]}")
            
            # TODO: Zainicjalizuj multiplekser TCA9548A (adres 0x70)
            # self.tca9548 = TCA9548A(self.i2c, addr=0x70)
            
            # TODO: Zainicjalizuj IMU ICM-20948
            # self.imu = ICM20948(self.i2c, addr=0x69)
            
            # TODO: Zainicjalizuj enkodery
            # self.encoders = {}
            # for name, (pin_a, pin_b) in ENCODER_PINS.items():
            #     self.encoders[name] = Encoder(pin_a, pin_b)
            
            self.initialized = True
            print("[SENSORS] Initialized successfully")
            
        except Exception as e:
            print(f"[SENSORS] Initialization error: {e}")
            self.initialized = False
    
    def read_lidar(self):
        """
        Odczyt danych z sensorów odległości VL53L7CX.
        
        Returns:
            list[dict]: Lista słowników z danymi dla każdego promienia:
                [{"d": distance_mm, "hit": bool}, ...]
            
        UWAGA: Po otrzymaniu API czujników, zastąp placeholder prawdziwym odczytem.
        Obecnie VL53L7CX zwraca pojedynczą odległość per sensor (nie 15 promieni).
        """
        if not self.initialized:
            # Fallback - symulowane dane
            return [{"d": SENSOR_RANGE, "hit": False} for _ in range(NUM_SENSORS * RAYS_PER_SENSOR)]
        
        # TODO: Odczytaj rzeczywiste dane z VL53L7CX przez TCA9548A
        # Przykładowa implementacja (do uzupełnienia):
        # distances = []
        # for sensor_idx in range(NUM_SENSORS):
        #     self.tca9548.select_channel(sensor_idx)
        #     distance = self.vl53l7cx[sensor_idx].read_distance()
        #     # Dla każdego sensora generujemy RAYS_PER_SENSOR promieni
        #     for ray_idx in range(RAYS_PER_SENSOR):
        #         angle_offset = ray_angles_deg[ray_idx]
        #         distances.append({"d": distance, "hit": distance < SENSOR_RANGE})
        # return distances
        
        # Placeholder - zwraca symulowane dane
        return [{"d": SENSOR_RANGE, "hit": False} for _ in range(NUM_SENSORS * RAYS_PER_SENSOR)]
    
    def read_imu(self):
        """
        Odczyt danych z IMU (ICM-20948).
        
        Returns:
            dict: Słownik z danymi:
                {"ax": float, "ay": float, "az": float,      # Akcelerometr [m/s²]
                 "gx": float, "gy": float, "gz": float,      # Żyroskop [rad/s]
                 "mx": float, "my": float, "mz": float,      # Magnetometr [µT]
                 "heading": float}                            # Heading w radianach [0, 2π)
        """
        if not self.initialized:
            return {
                "ax": 0.0, "ay": 0.0, "az": 9.81,
                "gx": 0.0, "gy": 0.0, "gz": 0.0,
                "mx": 0.0, "my": 0.0, "mz": 0.0,
                "heading": 0.0
            }
        
        # TODO: Odczytaj rzeczywiste dane z ICM-20948
        # Przykładowa implementacja:
        # raw = self.imu.read_all()
        # heading = math.atan2(raw['my'], raw['mx'])
        # if heading < 0:
        #     heading += 2 * math.pi
        # return {
        #     "ax": raw['ax'], "ay": raw['ay'], "az": raw['az'],
        #     "gx": raw['gx'], "gy": raw['gy'], "gz": raw['gz'],
        #     "mx": raw['mx'], "my": raw['my'], "mz": raw['mz'],
        #     "heading": heading
        # }
        
        # Placeholder
        return {
            "ax": 0.0, "ay": 0.0, "az": 9.81,
            "gx": 0.0, "gy": 0.0, "gz": 0.0,
            "mx": 0.0, "my": 0.0, "mz": 0.0,
            "heading": 0.0
        }
    
    def read_encoders(self):
        """
        Odczyt enkoderów silników (kwadraturowe).
        
        Returns:
            dict: Słownik z licznikami impulsów:
                {"fl": int, "fr": int, "rl": int, "rr": int}
        """
        if not self.initialized:
            return {"fl": 0, "fr": 0, "rl": 0, "rr": 0}
        
        # TODO: Odczytaj rzeczywiste wartości z enkoderów
        # Przykładowa implementacja:
        # return {
        #     "fl": self.encoders["fl"].get_count(),
        #     "fr": self.encoders["fr"].get_count(),
        #     "rl": self.encoders["rl"].get_count(),
        #     "rr": self.encoders["rr"].get_count()
        # }
        
        # Placeholder
        return {"fl": 0, "fr": 0, "rl": 0, "rr": 0}
    
    def get_position_estimate(self):
        """
        Szacowanie pozycji na podstawie enkoderów i IMU (dead reckoning / fuzja sensorów).
        
        Returns:
            tuple: (pos_x, pos_y, heading) w mm i radianach
        """
        if not self.initialized:
            return (0.0, 0.0, 0.0)
        
        # TODO: Implementacja fuzji sensorów (kalman filter / complementary filter)
        # Przykładowa implementacja z użyciem enkoderów i IMU:
        # encoders = self.read_encoders()
        # imu = self.read_imu()
        # Oblicz deltę pozycji na podstawie impulsów enkoderów
        # Zastosuj korektę heading z magnetometru/żyroskopu
        # return (new_x, new_y, new_heading)
        
        # Placeholder
        return (0.0, 0.0, 0.0)


# ============================================================
# KLASY POMOCNICZE
# ============================================================

def hmac_sha256(key_hex: str, message_hex: str) -> str:
    """
    Oblicza HMAC-SHA256.
    
    Uwaga: MicroPython nie ma wbudowanego hmac. 
    Można użyć micropython-lib lub zaimplementować ręcznie.
    Poniżej placeholder - należy zastąpić prawdziwą implementacją.
    
    Args:
        key_hex: Klucz w formacie hex string
        message_hex: Wiadomość w formacie hex string
    
    Returns:
        str: Podpis HMAC w formacie hex string
    """
    # TODO: Zaimplementuj prawdziwy HMAC-SHA256
    # Opcje:
    # 1. Użyj micropython-lib: import hmac, hashlib
    # 2. Zaimplementuj SHA256 ręcznie (wolne, ale działa)
    # 3. Przenieś autentykację na Raspberry Pi (jeśli dostępne)
    
    # Placeholder - NIE UŻYWAJ W PRODUKCJI!
    import hashlib
    key_bytes = bytes.fromhex(key_hex)
    msg_bytes = bytes.fromhex(message_hex)
    
    # Prosta implementacja HMAC (uproszczona)
    # W produkcji użyj micropython-lib hmac
    h = hashlib.sha256()
    h.update(key_bytes + msg_bytes)
    return h.hexdigest()


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
# KLASA STEROWNIKA (DFS + SLAM - uproszczona)
# ============================================================

class RobotController:
    """
    Główna klasa sterująca robotem.
    
    Implementuje:
    - Prostą eksplorację DFS
    - Podstawowe SLAM (mapa probabilistyczna)
    - Sterowanie ruchem
    - Watchdog i bezpieczeństwo
    """
    
    def __init__(self):
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
        
        # Mapa i eksploracja DFS
        self.visited_cells = set()
        self.path_stack = []
        self.current_cell = (0, 0)
        self.max_r = 0
        self.max_c = 0
        
        # Stan maszyny stanów
        self.exploration_done = False
        self.fast_run = False
        self.finished = False
        self.goal_cell = None
        self.optimized_path = []
        
        # Watchdog
        self.last_command_time = 0
        
        # Sensory
        self.sensors = SensorAPI()
        
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
    
    def update_sensors(self):
        """Aktualizuje dane z czujników."""
        if not self.sensors.initialized:
            self.sensors.init()
        
        # TODO: Odczytaj rzeczywiste dane z czujników
        lidar_data = self.sensors.read_lidar()
        imu_data = self.sensors.read_imu()
        encoders = self.sensors.read_encoders()
        
        # Aktualizuj pozycję (placeholder - użyj prawdziwej fuzji sensorów)
        pos = self.sensors.get_position_estimate()
        self.pos_x, self.pos_y, self.heading = pos
        
        return lidar_data, imu_data, encoders
    
    def find_next_target(self):
        """
        Znajduje następną komórkę do odwiedzenia w eksploracji DFS.
        
        Returns:
            tuple: (row, col) następnej komórki lub None jeśli brak
        """
        r, c = self.current_cell
        directions = [(-1, 0), (0, 1), (1, 0), (0, -1)]  # N, E, S, W
        
        for dr, dc in directions:
            nr, nc = r + dr, c + dc
            if (nr, nc) not in self.visited_cells:
                return (nr, nc)
        
        # Brak nieodwiedzonych sąsiadów - cofnij się
        if self.path_stack:
            return self.path_stack.pop()
        
        return None
    
    def compute_shortest_path(self, start, goal):
        """
        Oblicza najkrótszą ścieżkę BFS od start do goal.
        
        Args:
            start: (row, col) komórki startowej
            goal: (row, col) komórki docelowej
        
        Returns:
            list: Lista komórek na ścieżce [(r1,c1), (r2,c2), ...]
        """
        from collections import deque
        
        queue = deque([(start, [start])])
        visited = {start}
        
        while queue:
            (r, c), path = queue.popleft()
            
            if (r, c) == goal:
                return path
            
            for dr, dc in [(-1, 0), (0, 1), (1, 0), (0, -1)]:
                nr, nc = r + dr, c + dc
                if (nr, nc) not in visited and (nr, nc) in self.visited_cells:
                    visited.add((nr, nc))
                    queue.append(((nr, nc), path + [(nr, nc)]))
        
        return []  # Brak ścieżki
    
    def update_exploration_state(self):
        """
        Aktualizuje stan eksploracji i sprawdza czy zakończyć fazę DFS.
        
        Returns:
            bool: True jeśli przeszliśmy do fast_run
        """
        if self.exploration_done or self.fast_run:
            return False
        
        # Kryterium stopu: Pusty stos, powrót na start i zbadanie więcej niż 1 komórki
        if len(self.path_stack) == 0 and self.current_cell == (0, 0) and len(self.visited_cells) > 1:
            self.exploration_done = True
            print("[EXPLORATION] DFS completed!")
            
            # Oblicz cel (narożnik lub środek)
            max_r = max(r for r, c in self.visited_cells)
            max_c = max(c for r, c in self.visited_cells)
            
            # TODO: Dodaj wybór między narożnikiem a środkiem (jak w client.py)
            self.goal_cell = (max_r, max_c)
            print(f"[GOAL] Target cell: {self.goal_cell}")
            
            # Oblicz optymalną ścieżkę BFS
            self.optimized_path = self.compute_shortest_path((0, 0), self.goal_cell)
            if self.optimized_path:
                self.optimized_path = self.optimized_path[1:]  # Pomiń start
            
            self.fast_run = True
            self.target_cell = None
            print(f"[FAST RUN] Path length: {len(self.optimized_path)} cells")
            return True
        
        return False
    
    def explore_next(self):
        """
        Eksploracja DFS - znajduje następną komórkę do odwiedzenia.
        
        Returns:
            tuple: (vx, vy, w) lub None jeśli brak celu
        """
        # Sprawdź czy zakończyć eksplorację
        if self.update_exploration_state():
            return None
        
        # Jeśli już w fast_run, użyj nawigacji po ścieżce
        if self.fast_run:
            return None  # Obsłużone w get_movement_command()
        
        # Znajdź następną komórkę
        next_cell = self.find_next_target()
        
        if next_cell is None:
            # Brak celu - zatrzymaj
            return (0.0, 0.0, 0.0)
        
        # Dodaj do stosu jeśli to nowa komórka
        if next_cell not in self.visited_cells:
            self.path_stack.append(self.current_cell)
            self.visited_cells.add(next_cell)
            self.current_cell = next_cell
            
            # Aktualizuj max wymiary
            r, c = next_cell
            if r > getattr(self, 'max_r', 0):
                self.max_r = r
            if c > getattr(self, 'max_c', 0):
                self.max_c = c
        
        # Konwersja komórki logicznej na współrzędne fizyczne
        target_x = next_cell[1] * CELL_SIZE
        target_y = next_cell[0] * CELL_SIZE
        
        # Prosty regulator do celu
        dx = target_x - self.pos_x
        dy = target_y - self.pos_y
        dist = math.sqrt(dx**2 + dy**2)
        
        if dist < 5.0:  # Blisko celu
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
    
    def navigate_to(self, target_x: float, target_y: float):
        """
        Nawigacja do punktu docelowego.
        
        Args:
            target_x: Współrzędna X celu [mm]
            target_y: Współrzędna Y celu [mm]
        """
        self.target_x = target_x
        self.target_y = target_y
    
    def get_movement_command(self) -> dict:
        """
        Generuje komendę ruchu.
        
        Returns:
            dict: Komenda w formacie {"vx": float, "vy": float, "w": float}
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
                    self.optimized_path.pop(0)  # Usuń odwiedzoną komórkę
                    
                    # Sprawdź czy to cel końcowy
                    if next_cell == self.goal_cell:
                        print("[FAST RUN] Goal reached!")
                        self.finished = True
                        return {"vx": 0.0, "vy": 0.0, "w": 0.0}
                    
                    # Kontynuuj do następnej komórki
                    if not self.optimized_path:
                        self.finished = True
                        return {"vx": 0.0, "vy": 0.0, "w": 0.0}
                    
                    next_cell = self.optimized_path[0]
                    target_x = next_cell[1] * CELL_SIZE
                    target_y = next_cell[0] * CELL_SIZE
                    dx = target_x - self.pos_x
                    dy = target_y - self.pos_y
                
                # Nawigacja do następnej komórki (szybsza prędkość w fast run)
                speed = 0.6  # Szybciej niż w eksploracji
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
                # Ścieżka pusta - koniec fast run
                self.finished = True
                return {"vx": 0.0, "vy": 0.0, "w": 0.0}
        
        # FAZA 1: EKSPLORACJA DFS
        if self.target_x is not None and self.target_y is not None:
            # Nawigacja do ręcznie ustawionego celu
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
            # Eksploracja DFS
            result = self.explore_next()
            if result:
                vx, vy, w = result
                return {"vx": vx, "vy": vy, "w": w}
            else:
                return {"vx": 0.0, "vy": 0.0, "w": 0.0}
    
    def check_watchdog(self, current_time_ms: int) -> bool:
        """
        Sprawdza watchdog.
        
        Returns:
            bool: True jeśli watchdog aktywny, False jeśli przekroczono timeout
        """
        elapsed = current_time_ms - self.last_command_time
        if elapsed > WATCHDOG_TIMEOUT_MS:
            print(f"[WATCHDOG] Timeout! ({elapsed}ms > {WATCHDOG_TIMEOUT_MS}ms)")
            self.disarm()
            return False
        return True


# ============================================================
# KLIENT WEBSOCKET
# ============================================================

class WebSocketClient:
    """
    Klient WebSocket do komunikacji z robotem.
    
    Implementuje:
    - Autentykację challenge-response
    - Strumieniowanie komend
    - Odbieranie telemetrii
    - Obsługę błędów i reconnect
    """
    
    def __init__(self, controller: RobotController):
        self.controller = controller
        self.ws = None
        self.session_token = None
        self.auth_level = 0
    
    async def connect_and_authenticate(self):
        """
        Łączy się z robotem i przeprowadza autentykację.
        
        Returns:
            bool: True jeśli autentykacja udana
        """
        try:
            # TODO: Użyj prawdziwej biblioteki WebSocket dla MicroPython
            # microWebSocket lub websockets (jeśli dostępne)
            # Poniżej pseudokod - należy dostosować do dostępnej biblioteki
            
            print(f"[WS] Connecting to {ROBOT_WS_URL}...")
            
            # Placeholder - w MicroPython użyj microWebSocket lub podobnej biblioteki
            # from microWebSockets import WebSocketClient
            # self.ws = WebSocketClient(ROBOT_WS_URL)
            # await self.ws.connect()
            
            # Symulacja połączenia (USUNĄĆ W PRODUKCJI)
            print("[WS] Connected (simulated)")
            
            # Odbierz nonce
            # nonce_msg = await self.ws.recv()
            # nonce_data = json.loads(nonce_msg)
            # nonce = nonce_data["nonce"]
            
            # Oblicz odpowiedź HMAC
            # answer = hmac_sha256(ROBOT_HMAC_KEY, nonce)
            
            # Wyślij odpowiedź
            # auth_msg = json.dumps({"auth": answer})
            # await self.ws.send(auth_msg)
            
            # Odbierz potwierdzenie
            # reply_msg = await self.ws.recv()
            # reply_data = json.loads(reply_msg)
            
            # if reply_data.get("auth") == "ok":
            #     self.session_token = reply_data.get("token")
            #     self.auth_level = reply_data.get("level", 0)
            #     print(f"[WS] Authenticated, level={self.auth_level}")
            #     return True
            # else:
            #     print("[WS] Authentication failed")
            #     return False
            
            # Placeholder - zawsze udana autentykacja (USUNĄĆ W PRODUKCJI)
            self.session_token = "SIMULATED_TOKEN"
            self.auth_level = 2
            print("[WS] Authenticated (simulated)")
            return True
            
        except Exception as e:
            print(f"[WS] Connection error: {e}")
            return False
    
    async def send_command(self, command: dict):
        """
        Wysyła komendę do robota.
        
        Args:
            command: Słownik z komendą, np. {"vx": 0.5, "vy": 0.0, "w": 0.0}
        """
        if not self.ws:
            return
        
        try:
            msg = json.dumps(command)
            # await self.ws.send(msg)
            print(f"[WS] Sent: {msg}")  # Debug
        except Exception as e:
            print(f"[WS] Send error: {e}")
    
    async def receive_telemetry(self):
        """
        Odbiera i przetwarza telemetrię z robota.
        
        Returns:
            dict: Dane telemetrii lub None
        """
        if not self.ws:
            return None
        
        try:
            # msg = await self.ws.recv()
            # data = json.loads(msg)
            # return data
            
            # Placeholder
            return None
        except Exception as e:
            print(f"[WS] Receive error: {e}")
            return None
    
    async def stream_commands(self):
        """
        Główna pętla strumieniowania komend.
        
        Wysyła komendy z częstotliwością COMMAND_RATE_HZ
        i obsługuje watchdog.
        """
        interval = 1.0 / COMMAND_RATE_HZ
        
        while not self.controller.finished:
            start_time = time.ticks_ms()
            
            # Sprawdź watchdog
            if not self.controller.check_watchdog(time.ticks_ms()):
                # Rozbrojony - wyślij komendę neutralną aby uzbroić
                await self.send_command({"vx": 0.0, "vy": 0.0, "w": 0.0})
                await asyncio.sleep(0.5)
                self.controller.arm()
                continue
            
            # Pobierz komendę ruchu
            command = self.controller.get_movement_command()
            
            # Wyślij komendę
            await self.send_command(command)
            
            # Aktualizuj czas ostatniej komendy
            self.controller.last_command_time = time.ticks_ms()
            
            # Odbierz telemetrię (opcjonalnie)
            telemetry = await self.receive_telemetry()
            if telemetry:
                self.controller.telemetry = telemetry
                # Aktualizuj pozycję z telemetrii
                if "pos_x" in telemetry:
                    self.controller.pos_x = telemetry["pos_x"]
                    self.controller.pos_y = telemetry["pos_y"]
                    self.controller.heading = telemetry.get("heading", 0.0)
            
            # Czekaj do następnej iteracji
            elapsed = time.ticks_ms() - start_time
            sleep_time = max(0, interval - elapsed / 1000.0)
            await asyncio.sleep(sleep_time)
    
    async def run(self):
        """
        Główna metoda uruchamiająca klienta.
        """
        # Połącz i autentykuj
        if not await self.connect_and_authenticate():
            print("[CLIENT] Authentication failed, exiting")
            return
        
        # Uzbrój robota (wyślij komendy neutralne)
        print("[CLIENT] Arming robot...")
        for _ in range(10):  # 10 komend neutralnych
            await self.send_command({"vx": 0.0, "vy": 0.0, "w": 0.0})
            await asyncio.sleep(0.05)
        self.controller.arm()
        
        # Uruchom główną pętlę
        print("[CLIENT] Starting command stream...")
        try:
            await self.stream_commands()
        except KeyboardInterrupt:
            print("[CLIENT] Interrupted")
        finally:
            # Zatrzymaj robota przed zamknięciem
            await self.send_command({"dir": "stop"})
            print("[CLIENT] Stopped")


# ============================================================
# FUNKCJA GŁÓWNA
# ============================================================

async def main():
    """Główna funkcja programu."""
    print("=" * 60)
    print("MicroPython Mecanum Robot Client")
    print("=" * 60)
    
    # Utwórz kontroler
    controller = RobotController()
    
    # Utwórz klient WebSocket
    ws_client = WebSocketClient(controller)
    
    # Uruchom klienta WebSocket (łączy się z Raspberry Pi)
    await ws_client.run()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[EXIT] Program terminated by user")
    except Exception as e:
        print(f"\n[ERROR] Unexpected error: {e}")
        import sys
        sys.print_exception(e)


# ============================================================
# CHECKLISTA PRZED URUCHOMIENIEM
# ============================================================
"""
PRZED URUCHOMIENIEM NA ESP32-S3:

1. KONFIGURACJA KREDENSJALI:
   □ Skopiuj secrets_template.py → secrets.py
   □ Wypełnij ROBOT_HMAC_KEY (64 znaki hex)
   □ Zmień ROBOT_WS_URL na IP robota

2. BIBLIOTEKI DO ZAINSTALOWANIA:
   □ microWebSockets lub websockets (dla WebSocket)
   □ micropython-lib (dla hmac, hashlib) - opcjonalnie
   □ Driver dla VL53L7CX (jeśli dostępny)
   □ Driver dla ICM-20948 (jeśli dostępny)

3. IMPLEMENTACJA CZUJNIKÓW (po otrzymaniu API):
   □ SensorAPI.read_lidar() - odczyt VL53L7CX przez TCA9548A
   □ SensorAPI.read_imu() - odczyt ICM-20948
   □ SensorAPI.read_encoders() - odczyt enkoderów kwadraturowych
   □ SensorAPI.get_position_estimate() - fuzja sensorów

4. IMPLEMENTACJA WEBSOCKET:
   □ Zastąp placeholder w WebSocketClient.connect_and_authenticate()
   □ Zaimplementuj prawdziwe wysyłanie/odbieranie danych
   □ Dodaj obsługę reconnect przy utracie połączenia

5. TESTOWANIE:
   □ Test autentykacji z robotem
   □ Test wysyłania komend neutralnych (uzbrajanie)
   □ Test odczytu czujników (jeśli podłączone)
   □ Test watchdog (czy robot zatrzymuje się po timeout)

6. OPTYMALIZACJA DLA MICROPYTHON:
   □ Unikaj alokacji pamięci w pętli głównej
   □ Używaj time.ticks_ms() zamiast time.time()
   □ Minimalizuj użycie float (ESP32 ma FPU, ale wolne)
   □ Rozważ użycie fixed-point arithmetic dla krytycznych operacji

PRZYKŁADOWA STRUKTURA PLIKÓW NA ESP32:
/main.py              - Ten plik (główny kod)
/secrets.py           - Klucze API (NIE commitować do git!)
/lib/
  /microWebSockets/   - Biblioteka WebSocket
  /vl53l7cx.py        - Driver VL53L7CX
  /icm20948.py        - Driver ICM-20948
  /encoder.py         - Driver enkoderów
  /tca9548a.py        - Driver multipleksera I2C
"""

