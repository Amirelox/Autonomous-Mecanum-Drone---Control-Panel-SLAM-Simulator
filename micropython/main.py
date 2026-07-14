"""
Raspberry Pi Client for Mecanum Robot with ESP32-S3
====================================================
Klient dla Raspberry Pi łączący się z ESP32-S3 przez WebSocket.
RPi wykonuje całą logikę (DFS, SLAM, nawigacja), a ESP32 tylko:
- Steruje silnikami
- Odczytuje czujniki (placeholder)
- Wysyła dane telemetryczne

Wymagane biblioteki Python 3:
- asyncio (wbudowane)
- json (wbudowane)
- websockets (pip install websockets)
- numpy (pip install numpy)
"""

import asyncio
import json
import time
import math
import hmac
import hashlib
from typing import Optional, Dict, Any, List, Tuple

# Spróbuj załadować klucze z secrets.py (opcjonalne)
try:
    from secrets import ROBOT_HMAC_KEY, ESP32_WS_URL
    print("[CONFIG] Loaded credentials from secrets.py")
except ImportError:
    # Domyślne wartości (zmień przed uruchomieniem!)
    ROBOT_HMAC_KEY = "YOUR_64_CHAR_HEX_KEY_HERE"
    ESP32_WS_URL = "ws://192.168.4.1:8765/ws"
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

# Watchdog timeout [ms]
WATCHDOG_TIMEOUT_MS = 350

# Częstotliwość wysyłania komend [Hz]
COMMAND_RATE_HZ = 25

# ============================================================
# KLIENT WEBSOCKET DO ESP32
# ============================================================

class ESP32WebSocketClient:
    """
    Klient WebSocket łączący Raspberry Pi z ESP32-S3.
    
    Implementuje:
    - Autentykację challenge-response (HMAC-SHA256)
    - Odbieranie danych z czujników od ESP32
    - Wysyłanie komend sterujących do ESP32
    """
    
    def __init__(self, host: str = "192.168.4.1", port: int = 8765, secret_key: str = ""):
        self.host = host
        self.port = port
        self.secret_key = secret_key
        self.ws = None
        self.session_token = None
        self.auth_level = 0
        self.connected = False
        self.latest_sensor_data = None
    
    async def connect(self) -> bool:
        """
        Łączy się z ESP32 i przeprowadza autentykację.
        
        Returns:
            bool: True jeśli połączenie i autentykacja udane
        """
        try:
            import websockets
            
            ws_url = f"ws://{self.host}:{self.port}"
            print(f"[WS] Connecting to {ws_url}...")
            
            self.ws = await asyncio.wait_for(
                websockets.connect(ws_url),
                timeout=5.0
            )
            
            print("[WS] Connected, waiting for nonce...")
            
            # Odbierz nonce od ESP32
            nonce_msg = await self.ws.recv()
            nonce_data = json.loads(nonce_msg)
            nonce = nonce_data["nonce"]
            
            print(f"[WS] Received nonce: {nonce[:16]}...")
            
            # Oblicz odpowiedź HMAC-SHA256
            key_bytes = bytes.fromhex(self.secret_key)
            nonce_bytes = bytes.fromhex(nonce)
            answer = hmac.new(key_bytes, nonce_bytes, hashlib.sha256).hexdigest()
            
            # Wyślij odpowiedź
            auth_msg = json.dumps({"auth": answer})
            await self.ws.send(auth_msg)
            
            print("[WS] Sent authentication response")
            
            # Odbierz potwierdzenie
            while True:
                reply_msg = await self.ws.recv()
                reply_data = json.loads(reply_msg)
                
                if "auth" in reply_data:
                    if reply_data["auth"] == "ok":
                        self.session_token = reply_data.get("token")
                        self.auth_level = reply_data.get("level", 0)
                        self.connected = True
                        print(f"[WS] ✓ Authenticated! Level: {self.auth_level}")
                        return True
                    else:
                        print(f"[WS] ✗ Authentication failed")
                        return False
                        
        except Exception as e:
            print(f"[WS] ✗ Connection error: {e}")
            return False
    
    async def disconnect(self):
        """Zamyka połączenie WebSocket."""
        if self.ws:
            await self.ws.close()
            self.connected = False
            print("[WS] Disconnected")
    
    async def send_command(self, command: dict):
        """Wysyła komendę sterującą do ESP32."""
        if not self.ws or not self.connected:
            return
        
        try:
            msg = json.dumps(command)
            await self.ws.send(msg)
        except Exception as e:
            print(f"[WS] Send error: {e}")
    
    async def receive_sensor_data(self):
        """
        Odbiera dane z czujników od ESP32.
        
        Returns:
            dict: Dane z czujników lub None
        """
        if not self.ws or not self.connected:
            return None
        
        try:
            msg = await asyncio.wait_for(self.ws.recv(), timeout=0.1)
            data = json.loads(msg)
            
            # Sprawdź czy to dane z czujników (nie telemetria)
            if "sensors" in data or "lidar" in data or "imu" in data:
                self.latest_sensor_data = data
                return data
            
            return None
            
        except asyncio.TimeoutError:
            return None
        except Exception as e:
            print(f"[WS] Receive error: {e}")
            return None
    
    async def close(self):
        """Zamyka połączenie WebSocket."""
        if self.ws:
            await self.ws.close()
            self.connected = False


# ============================================================
# PLACEHOLDER: API CZUJNIKÓW (odczyt przez ESP32)
# ============================================================
# TODO: Po otrzymaniu dokumentacji API czujników, wypełnij tę klasę

class SensorAPI:
    """
    Klasa obsługująca czujniki robota przez ESP32.
    
    Na Raspberry Pi czujniki są fizycznie podłączone do ESP32.
    Ta klasa odbiera dane od ESP32 przez WebSocket.
    """
    
    def __init__(self, ws_client: ESP32WebSocketClient = None):
        self.initialized = False
        self.last_read_time = 0
        self.ws_client = ws_client
        
    def init(self, ws_client: ESP32WebSocketClient = None):
        """Inicjalizacja połączenia z ESP32."""
        if ws_client:
            self.ws_client = ws_client
        self.initialized = True
        print("[SENSORS] Initialized (will receive data from ESP32)")
    
    def read_lidar(self):
        """
        Odczyt danych z sensorów odległości VL53L7CX (przez ESP32).
        
        Returns:
            list[dict]: Lista słowników z danymi dla każdego promienia
        """
        # TODO: Gdy API będzie gotowe, odbierz dane z ESP32
        # Placeholder - symulowane dane
        return [{"d": 200.0, "hit": False} for _ in range(NUM_SENSORS * RAYS_PER_SENSOR)]
    
    def read_imu(self):
        """
        Odczyt danych z IMU ICM-20948 (przez ESP32).
        
        Returns:
            dict: Słownik z danymi IMU
        """
        # TODO: Gdy API będzie gotowe, odbierz dane z ESP32
        # Placeholder
        return {
            "ax": 0.0, "ay": 0.0, "az": 9.81,
            "gx": 0.0, "gy": 0.0, "gz": 0.0,
            "mx": 0.0, "my": 0.0, "mz": 0.0,
            "heading": 0.0
        }
    
    def read_encoders(self):
        """
        Odczyt enkoderów silników (przez ESP32).
        
        Returns:
            dict: Słownik z licznikami impulsów
        """
        # TODO: Gdy API będzie gotowe, odbierz dane z ESP32
        # Placeholder
        return {"fl": 0, "fr": 0, "rl": 0, "rr": 0}
    
    def get_position_estimate(self):
        """
        Szacowanie pozycji na podstawie danych od ESP32.
        
        Returns:
            tuple: (pos_x, pos_y, heading)
        """
        # TODO: Implementacja fuzji sensorów gdy API będzie gotowe
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
    
    def __init__(self, ws_client: ESP32WebSocketClient = None):
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
        
        # Połączenie z ESP32
        self.ws_client = ws_client
        
        # Sensory (przez ESP32)
        self.sensors = SensorAPI(ws_client)
        
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
        """Aktualizuje dane z czujników przez ESP32 i aktualizuje mapę logiczną."""
        if not self.sensors.initialized:
            self.sensors.init()
        
        # Odczytaj dane z ESP32 (placeholder - gdy API będzie gotowe)
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
        
        # TODO: Gdy API czujników będzie gotowe, użyj LiDAR do wykrywania ścian
        # i aktualizuj logic_map[r][c] = 1 dla ścian
        
        return lidar_data, imu_data, encoders
    
    def find_next_target(self):
        """
        Znajduje następną komórkę do odwiedzenia w eksploracji DFS.
        Sprawdza mapę logiczną ścian przed podjęciem decyzji.
        
        Returns:
            tuple: (row, col) następnej komórki lub None jeśli brak
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
    
    def run_exploration_step(self):
        """
        Wykonuje jeden krok eksploracji/nawigacji i wysyła komendę do ESP32.
        
        Ta metoda:
        1. Aktualizuje sensory z ESP32
        2. Oblicza następną komendę ruchu (DFS lub fast run)
        3. Wysyła komendę do ESP32 przez WebSocket
        4. Aktualizuje watchdog
        """
        if self.estop or not self.armed:
            # Rozbrojony - wyślij komendę neutralną
            if self.ws_client and self.ws_client.connected:
                asyncio.create_task(self.ws_client.send_command({"vx": 0.0, "vy": 0.0, "w": 0.0}))
            return
        
        # Aktualizuj sensory
        self.update_sensors()
        
        # Pobierz komendę ruchu
        command = self.get_movement_command()
        
        # Wyślij do ESP32
        if self.ws_client and self.ws_client.connected:
            asyncio.create_task(self.ws_client.send_command(command))
        
        # Aktualizuj watchdog
        self.last_command_time = int(time.time() * 1000)
    
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
            # Połączenie z ESP32 przez WebSocket
            print(f"[WS] Connecting to {self.host}:{self.port}...")
            
            # Placeholder - w produkcji użyj prawdziwej biblioteki WebSocket
            # import websockets
            # self.ws = await websockets.connect(f"ws://{self.host}:{self.port}")
            
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
    print("Mecanum Robot Client (Raspberry Pi)")
    print("=" * 60)
    
    # Utwórz klienta WebSocket do ESP32
    ws_client = ESP32WebSocketClient(
        host="192.168.4.1",  # Domyślny IP ESP32 w trybie AP
        port=8765,
        secret_key="robot_secret_2024"  # Zmień na swój klucz!
    )
    
    # Połącz z ESP32
    if not await ws_client.connect():
        print("[ERROR] Nie można połączyć się z ESP32")
        return
    
    # Utwórz kontroler robota
    controller = RobotController(ws_client)
    
    # Uruchom pętlę sterowania
    try:
        while True:
            # Aktualizuj sensory
            lidar_data, imu_data, encoders = controller.update_sensors()
            
            # Wykonaj logikę eksploracji lub nawigacji
            controller.run_exploration_step()
            
            # Czekaj przed następną iteracją
            await asyncio.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[EXIT] Program terminated by user")
    finally:
        await ws_client.disconnect()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[EXIT] Program terminated by user")
    except Exception as e:
        print(f"\n[ERROR] Unexpected error: {e}")
        import sys
        sys.print_exception(e)


