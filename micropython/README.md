# MicroPython Client dla ESP32-S3 Mecanum Robot

## 📋 Opis

Szablon implementacji klienta MicroPython dla robota z napędem mecanum na bazie ESP32-S3. Implementuje pełną logikę komunikacji z robotem, autentykację WebSocket, **dashboard HTTP** oraz strukturę pod przyszłe API czujników.

## 🎯 Funkcjonalności

### ✅ Zaimplementowane
- **Autentykacja WebSocket** (challenge-response z HMAC-SHA256)
- **Strumieniowanie komend** (20-50 Hz)
- **Watchdog** (350ms timeout)
- **Bezpieczeństwo** (E-stop, arming/disarming)
- **Mikser kinematyczny** (konwersja vx/vy/w na 4 koła)
- **Placeholder API czujników** (LiDAR, IMU, enkodery)
- **Pełna logika dwuetapowa**:
  - Faza 1: Eksploracja DFS (odwiedzanie wszystkich komórek)
  - Faza 2: Fast Run (optymalna ścieżka BFS do celu)
- **Komunikacja z Raspberry Pi** (ESP32 jako klient WebSocket)

### 🌐 Dashboard (Raspberry Pi)
Dashboard NiceGUI działa na **Raspberry Pi** (nie na ESP32) i zawiera:
- Wizualizację mapy eksploracji w czasie rzeczywistym
- Pozycję robota i dane z czujników
- Sterowanie (START, STOP, RESET)
- Dostęp przez `http://localhost:8080`

### 🔧 Do uzupełnienia po otrzymaniu API czujników
1. **SensorAPI.read_lidar()** - odczyt VL53L7CX przez I2C
2. **SensorAPI.read_imu()** - odczyt ICM-20948 przez I2C/SPI
3. **SensorAPI.read_encoders()** - odczyt enkoderów JGA25-370
4. **SensorAPI.get_position_estimate()** - fuzja sensorów (Kalman/complementary filter)
5. **Prawdziwa implementacja HMAC-SHA256** (micropython-lib lub własna)
6. **Biblioteka WebSocket** (microWebSocket lub podobna)

## �️ Architektura systemu

```
┌─────────────────────┐         WebSocket          ┌──────────────────┐
│   ESP32-S3-Zero     │ ◄────────────────────────► │  Raspberry Pi    │
│                     │      (port 8765)           │                  │
│ • Sterowanie        │                            │ • Dashboard HTTP │
│   silnikami         │                            │   (NiceGUI)      │
│ • Odczyt czujników  │                            │ • Logika DFS/SLAM│
│ • Watchdog          │                            │ • Autentykacja   │
│ • Komendy ruchu     │                            │ • Mapa           │
└─────────────────────┘                            └──────────────────┘
       ↑                                                    ↑
       │                                                    │
   UART/I2C/SPI                                     http://localhost:8080
   (czujniki)                                         (przeglądarka)
```

**ESP32-S3:**
- Łączy się z Raspberry Pi przez WebSocket
- Wysyła dane telemetryczne (pozycja, czujniki)
- Odbiera komendy ruchu (vx, vy, w)
- Implementuje watchdog i bezpieczeństwo

**Raspberry Pi:**
- Hostuje dashboard NiceGUI (port 8080)
- Zarządza logiką eksploracji i nawigacji
- Przetwarza dane SLAM
- Wysyła komendy do ESP32

## 🚀 Instalacja

### Hardware
- ESP32-S3-Zero (lub inny ESP32-S3)
- Moduł WiFi (wbudowany w ESP32-S3)
- Czujniki (do podłączenia):
  - 6x VL53L7CX (ToF) lub inny LiDAR
  - ICM-20948 (IMU 9-DOF)
  - Enkodery przy silnikach JGA25-370

### Software
- MicroPython v1.20+ dla ESP32-S3
- Biblioteki:
  - `uasyncio` (wbudowane)
  - `ujson` (wbudowane)
  - `hashlib` (wbudowane, ale bez HMAC)
  - Opcjonalnie: `micropython-lib` (dla hmac)

## 🚀 Instalacja

### 1. Flash MicroPython na ESP32-S3

```bash
# Pobierz firmware MicroPython dla ESP32-S3
# https://micropython.org/download/esp32/

# Flashuj używając esptool.py
esptool.py --port COM3 erase_flash
esptool.py --port COM3 write_flash -z 0x1000 esp32s3-*.bin
```

### 2. Zainstaluj wymagane biblioteki

```bash
# Połącz się z REPL ESP32 przez serial
# Ctrl+B aby wejść do trybu paste mode

# Zainstaluj micropython-lib (opcjonalnie, dla HMAC)
import upip
upip.install('hmac')
upip.install('hashlib')
```

### 3. Skonfiguruj sekrety

```bash
# Skopiuj template
cp secrets_template.py secrets.py

# Edytuj secrets.py i wypełnij prawdziwymi wartościami
# ROBOT_HMAC_KEY = "twój_64_znakowy_klucz_hex"
# ROBOT_WS_URL = "ws://192.168.1.100/ws"
```

### 4. Wgraj pliki na ESP32

```bash
# Użyj ampy, rshell lub Thonny IDE
ampy --port COM3 put micropython_client.py
ampy --port COM3 put secrets.py
ampy --port COM3 put config.py  # jeśli masz osobny plik konfiguracyjny
```

## 🔌 Podłączenie czujników

### I2C Bus (przykładowe piny ESP32-S3-Zero)
```
SDA -> GPIO 21
SCL -> GPIO 22
```

### SPI Bus (dla IMU jeśli potrzeba)
```
CS   -> GPIO 5
SCK  -> GPIO 18
MOSI -> GPIO 23
MISO -> GPIO 19
```

### Enkodery (GPIO z interrupt)
```
FL encoder A/B -> GPIO 32/33
FR encoder A/B -> GPIO 25/26
RL encoder A/B -> GPIO 27/14
RR encoder A/B -> GPIO 12/13
```

## 📝 Struktura kodu

```
micropython_client.py
├── Konfiguracja (stałe fizyczne, parametry)
├── SensorAPI (placeholder pod czujniki)
│   ├── read_lidar()      # TODO: VL53L7CX
│   ├── read_imu()        # TODO: ICM-20948
│   ├── read_encoders()   # TODO: enkodery
│   └── get_position_estimate()  # TODO: fuzja sensorów
├── RobotController (logika sterowania)
│   ├── arm/disarm
│   ├── emergency_stop
│   ├── explore_next()    # DFS placeholder
│   ├── navigate_to()     # Nawigacja do punktu
│   └── get_movement_command()
├── WebSocketClient (komunikacja)
│   ├── connect_and_authenticate()
│   ├── send_command()
│   ├── receive_telemetry()
│   └── stream_commands()
└── main() (punkt wejścia)
```

## 🔐 Bezpieczeństwo

### Klucz HMAC
- **NIGDY** nie commituj `secrets.py` do Gita
- Dodaj `secrets.py` do `.gitignore`
- Przechowuj klucz w bezpiecznym miejscu
- Rotuj klucz okresowo

### Watchdog
- Robot zatrzymuje się po 350ms bez komend
- Wysyłaj komendy z częstotliwością 20-50 Hz
- Po rozbrojeniu wyślij serię komend neutralnych aby uzbroić

### E-Stop
- Wyślij `{"cmd": "estop"}` aby natychmiast zatrzymać
- Następnie `{"cmd": "resume"}` + komendy neutralne aby wznowić

## 🛠️ Rozwiązywanie problemów

### Problem: Autentykacja nie przechodzi
**Rozwiązanie:**
1. Sprawdź czy klucz HMAC jest poprawny (64 znaki hex)
2. Upewnij się że używasz surowych bajtów (bytes.fromhex)
3. Sprawdź czas systemowy ESP32 (nonce może wygasnąć)

### Problem: Robot nie reaguje na komendy
**Rozwiązanie:**
1. Sprawdź telemetrię: `armed` powinno być 1
2. Jeśli `armed: 0`, wyślij komendy neutralne przez 0.5s
3. Sprawdź `age_ms` w telemetrii - jeśli rośnie, strumień jest za wolny

### Problem: Brak biblioteki WebSocket
**Rozwiązanie:**
1. Użyj `microWebSocket`: `pip install microWebSockets`
2. Lub zaimplementuj ręcznie na socketach TCP
3. Lub przenieś autentykację na Raspberry Pi

### Problem: HMAC-SHA256 niedostępny
**Rozwiązanie:**
1. Zainstaluj `micropython-lib`: `upip.install('hmac')`
2. Lub zaimplementuj SHA256 ręcznie (wolne ale działa)
3. Lub użyj pre-computed tokenów (mniej bezpieczne)

## 📊 Telemetria

Robot wysyła ramkę stanu ~5 Hz:
```json
{
  "armed": 1,
  "estop": 0,
  "src": 1,
  "speed": 0.50,
  "age_ms": 18,
  "sta": 1,
  "heap": 150000,
  "rst": 1
}
```

| Pole | Znaczenie |
|------|-----------|
| armed | 1 = uzbrojony, 0 = wyślij komendy neutralne |
| estop | 1 = zatrzymanie awaryjne aktywne |
| src | Kto steruje: 0=nikt, 1=WebSocket, 2=UART |
| speed | Maksymalna prędkość koła [0-1] |
| age_ms | Czas od ostatniej komendy [ms] |
| sta | Połączenie z WiFi zewnętrznej |
| heap | Wolna pamięć RAM [bajty] |
| rst | Powód resetu: 1=zasilanie, 4=panic, 5/6=watchdog |

## 🔄 Roadmapa

### Po otrzymaniu API czujników:
1. [ ] Zaimplementuj `SensorAPI.read_lidar()` dla VL53L7CX
2. [ ] Zaimplementuj `SensorAPI.read_imu()` dla ICM-20948
3. [ ] Zaimplementuj odczyt enkoderów
4. [ ] Dodaj filtr Kalmana / complementary filter
5. [ ] Zaimplementuj pełny DFS jak w `client.py`
6. [ ] Dodaj SLAM z mapą probabilistyczną
7. [ ] Testy integracyjne z rzeczywistym robotem

## 📚 Zasoby

- [MicroPython Documentation](https://docs.micropython.org/)
- [ESP32-S3 Technical Reference](https://www.espressif.com/en/products/socs/esp32-s3)
- [API Dokumentacja Robota](api.md)
- [micropython-lib](https://github.com/micropython/micropython-lib)

## ⚠️ Uwagi

- Ten kod jest **SZABLONEM** - wymaga dostosowania do rzeczywistego hardware
- Placeholder'e oznaczone `TODO` muszą zostać zaimplementowane
- Testuj najpierw w symulacji przed uruchomieniem na rzeczywistym robocie
- Zawsze miej przycisk E-stop pod ręką podczas testów

## 📄 Licencja

Ten projekt jest częścią Autonomous Mecanum Drone Control Panel.
