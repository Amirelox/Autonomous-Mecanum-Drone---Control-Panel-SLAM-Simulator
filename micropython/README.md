# Klient Raspberry Pi dla ESP32-S3 Mecanum Robot

## 📋 Opis

Implementacja klienta w Python 3 dla Raspberry Pi, który komunikuje się z robotem na bazie ESP32-S3 przez WebSocket. Raspberry Pi działa jako **główny kontroler** (dashboard, SLAM, eksploracja), a ESP32 obsługuje niskopoziomowe sterowanie silnikami i odczyt czujników.

## 🎯 Funkcjonalności

### ✅ Zaimplementowane
- **Klient WebSocket** - połączenie z ESP32 (tryb AP lub WiFi)
- **Autentykacja challenge-response** (HMAC-SHA256)
- **Wysyłanie komend** (20-50 Hz) do ESP32
- **Odbieranie telemetry** od ESP32 (pozycja, status, czujniki)
- **Bezpieczeństwo** (E-stop, arming/disarming)
- **Logika eksploracji DFS** i nawigacji
- **SLAM** (mapa probabilistyczna)
- **Dashboard HTTP** (NiceGUI) - wizualizacja w przeglądarce

### 🔧 Do uzupełnienia po otrzymaniu API czujników
1. **ESP32WebSocketClient** - pełna implementacja odbierania danych z czujników
2. **SensorAPI** - integracja z danymi z ESP32 zamiast placeholderów
3. **Fuzja sensorów** - kalman filter/complementary filter dla pozycji

## 🏗️ Architektura systemu

```
┌─────────────────────┐         WebSocket          ┌──────────────────┐
│   ESP32-S3-Zero     │ ◄────────────────────────► │  Raspberry Pi    │
│   (Access Point)    │      ws://192.168.4.1/ws   │   (Main Brain)   │
│                     │       (port 8765)          │                  │
│ • Serwer WS         │                            │ • Dashboard HTTP │
│ • Sterowanie        │                            │   (NiceGUI)      │
│   silnikami         │                            │ • Logika DFS/SLAM│
│ • Odczyt czujników  │                            │ • Autentykacja   │
│ • Watchdog          │                            │ • Mapa           │
└─────────────────────┘                            └──────────────────┘
       ↑                                                    ↑
       │                                                    │
   UART/I2C/SPI                                     http://localhost:8080
   (czujniki)                                         (przeglądarka)
```

**Raspberry Pi (Główny kontroler):**
- Uruchamia dashboard HTTP (NiceGUI) na porcie 8080
- Łączy się z ESP32 przez WebSocket
- Wysyła komendy ruchu (vx, vy, w) do ESP32
- Odbiera dane telemetryczne (pozycja, status, czujniki)
- Implementuje logikę eksploracji DFS i SLAM

**Raspberry Pi:**
- Uruchamia dashboard HTTP (NiceGUI) na porcie 8080
- Łączy się z ESP32 przez WebSocket
- Wysyła komendy ruchu (vx, vy, w) do ESP32
- Odbiera dane telemetryczne (pozycja, status, czujniki)
- Implementuje logikę eksploracji DFS i SLAM

## 🚀 Instalacja

### Hardware
- Raspberry Pi (4B/5 lub Zero 2 W)
- ESP32-S3-Zero (lub inny ESP32-S3) - podłączony do robota
- Czujniki na robocie (podłączone do ESP32):
  - 6x VL53L7CX (ToF) z multiplekserem TCA9548A
  - ICM-20948 (IMU 9-DOF)
  - Enkodery przy silnikach JGA25-370

### Software (Raspberry Pi)
- Python 3.9+
- Biblioteki: `websockets`, `nicegui`, `numpy`
- System: Raspberry Pi OS (Debian-based)

## 🛠️ Konfiguracja

### 1. Zainstaluj zależności na Raspberry Pi

```bash
pip install websockets nicegui numpy
```

### 2. Skonfiguruj połączenie z ESP32

Edytuj plik `main.py` i ustaw:
```python
ws_client = ESP32WebSocketClient(
    host="192.168.4.1",  # IP ESP32 w trybie AP
    port=8765,
    secret_key="robot_secret_2024"  # Ten sam klucz co na ESP32!
)
```

### 3. Uruchom klienta

```bash
cd micropython
python main.py
```

Dashboard będzie dostępny pod adresem: http://localhost:8080

## 🔐 Bezpieczeństwo i Komunikacja

### Autentykacja WebSocket
Zgodnie z `api.md`, połączenie wymaga handshake'u:
1. ESP32 wysyła `{"nonce": "..."}`
2. Klient (RPi) oblicza HMAC-SHA256 i odsyła podpis
3. Po pomyślnej weryfikacji rozpoczyna się strumieniowanie danych

### Watchdog
- Robot zatrzymuje się po **350ms** bez nowej komendy
- Raspberry Pi musi wysyłać pakiety z częstotliwością **20-50 Hz**
- Po utracie połączenia robot przechodzi w stan rozbrojony (disarmed)

## 📊 Telemetria

ESP32 wysyła ramkę stanu do Raspberry Pi (~5 Hz):
```json
{
  "armed": 1,
  "estop": 0,
  "src": 1,
  "speed": 0.50,
  "age_ms": 18,
  "heap": 150000
}
```

## 🔄 Roadmapa

### Po otrzymaniu API czujników:
1. [ ] Zaimplementuj odbieranie danych z VL53L7CX przez WebSocket
2. [ ] Dodaj obsługę danych IMU ICM-20948
3. [ ] Zaimplementuj odczyt enkoderów kwadraturowych
4. [ ] Dodaj fuzję sensorów (kalman filter) dla pozycji

## ⚠️ Uwagi
- Ten kod jest przeznaczony do uruchomienia **na Raspberry Pi**.
- ESP32 musi być wcześniej skonfigurowany i wgrany z odpowiednim firmware.
- Upewnij się, że klucze HMAC są takie same na obu urządzeniach.
- Logika SLAM i dashboard znajdują się na **Raspberry Pi**.
- Upewnij się, że klucz HMAC w `secrets.py` jest identyczny z tym użytym w konfiguracji Raspberry Pi.
