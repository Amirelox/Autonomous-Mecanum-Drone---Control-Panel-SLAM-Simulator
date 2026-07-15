# ESP32 Brain Client dla Mecanum Robot (Architektura V1 - bez RPi)

## 📋 Opis

Implementacja klienta w **MicroPython** dla drugiego ESP32-S3 który działa jako **"mózg"** robota. Komunikuje się z pierwszym ESP32 (Hardware Controller) przez **UART**.

**Architektura V1 (bez Raspberry Pi):**
- **ESP32 #1 (Hardware Controller)**: steruje silnikami, czyta czujniki, serwuje API UART
- **ESP32 #2 (Brain)**: ten plik - algorytmy DFS, SLAM, nawigacja w MicroPythonie

## 🎯 Funkcjonalności

### ✅ Zaimplementowane
- **Klient UART** - połączenie z ESP32 Hardware (GPIO16/17, 115200 baud)
- **Wysyłanie komend** (20-50 Hz) do ESP32 Hardware
- **Odbieranie telemetry** od ESP32 Hardware (pozycja, status, czujniki)
- **Bezpieczeństwo** (E-stop, arming/disarming)
- **Logika eksploracji DFS** i nawigacji BFS
- **SLAM** (mapa logiczna ścian 41x41)
- **Kalibracja pozycji startowej** używając czujników ToF

### 🔧 Do uzupełnienia po otrzymaniu API czujników
1. **UARTHardwareClient** - pełna implementacja odbierania danych z czujników
2. **SensorAPI** - integracja z danymi z ESP32 Hardware zamiast placeholderów
3. **Fuzja sensorów** - kalman filter/complementary filter dla pozycji

## 🏗️ Architektura systemu

```
┌─────────────────────┐         UART          ┌──────────────────┐
│   ESP32-S3 #2       │ ◄───────────────────► │  ESP32-S3 #1     │
│   (Brain/Mózg)      │    GPIO16/17, 115200  │  (Hardware Ctrl) │
│                     │                       │                  │
│ • Algorytmy DFS/BFS │                       │ • Sterowanie     │
│ • SLAM              │                       │   silnikami      │
│ • Nawigacja         │                       │ • Odczyt czujn.  │
│ • Kalibracja        │                       │ • Watchdog       │
└─────────────────────┘                       └──────────────────┘
                                                       ↑
                                                       │
                                                  UART/I2C/SPI
                                                  (czujniki)
```

**ESP32 #2 (Brain/Mózg):**
- Uruchamia algorytm eksploracji DFS
- Oblicza ścieżki BFS do celów
- Wysyła komendy ruchu (vx, vy, w) przez UART
- Odbiera dane telemetryczne od ESP32 #1
- Implementuje logikę SLAM i mapę logiczną

**ESP32 #1 (Hardware Controller):**
- Steruje silnikami mecanum przez PWM
- Odczytuje czujniki (VL53L7CX, ICM-20948, enkodery)
- Serwuje API UART dla ESP32 #2
- Implementuje watchdog i bezpieczeństwo

## 🚀 Instalacja

### Hardware
- **ESP32-S3 #1 (Hardware Controller)**: podłączony do silników, czujników
- **ESP32-S3 #2 (Brain/Mózg)**: ten plik - algorytmy nawigacji
- Połączenie UART między ESP32: GPIO16 (RX) ↔ GPIO17 (TX), wspólna masa
- Czujniki na robocie (podłączone do ESP32 #1):
  - 6x VL53L7CX (ToF) z multiplekserem TCA9548A
  - ICM-20948 (IMU 9-DOF)
  - Enkodery przy silnikach JGA25-370

### Software
- **MicroPython v1.20+** dla ESP32-S3
- Biblioteki wbudowane: `uasyncio`, `ujson`, `machine`

## 🛠️ Konfiguracja

### 1. Flash MicroPython na obu ESP32

```bash
# Pobierz firmware MicroPython dla ESP32-S3
# https://micropython.org/download/esp32s3/

# Wyczyść flash
esptool.py --port COM3 erase_flash

# Wgraj firmware
esptool.py --port COM3 write_flash -z 0x1000 esp32s3-*.bin
```

### 2. Podłącz UART między ESP32

```
ESP32 #2 (Brain)          ESP32 #1 (Hardware)
GPIO17 (TX)  ──────────►  GPIO16 (RX)
GPIO16 (RX)  ◄──────────  GPIO17 (TX)
GND          ──────────►  GND
```

### 3. Wgraj pliki na ESP32 #2 (Brain)

```bash
# Użyj Thonny IDE lub ampy
ampy --port COM4 put micropython/esp32_brain_main.py /main.py
```

### 4. Uruchom program

Po wgraniu pliku, ESP32 automatycznie uruchomi `main.py`.
Możesz monitorować przez serial monitor (115200 baud).

## 🔐 Bezpieczeństwo i Komunikacja

### Protokół UART
Komunikacja między ESP32 odbywa się przez JSON linia po linii:
```json
{"cmd": "set_motors", "fl": 0.5, "fr": 0.5, "rl": 0.5, "rr": 0.5}\n
{"cmd": "get_sensors"}\n
```

### Watchdog
- Robot zatrzymuje się po **350ms** bez nowej komendy
- ESP32 Brain musi wysyłać pakiety z częstotliwością **20-50 Hz**
- Po utracie połączenia robot przechodzi w stan rozbrojony (disarmed)

## 📊 Telemetria

ESP32 Hardware wysyła dane do ESP32 Brain (~10 Hz):
```json
{
  "lidar": [{"d": 1500.0, "hit": true}, ...],
  "imu": {"ax": 0.0, "ay": 0.0, "az": 9.81, ...},
  "encoders": {"fl": 1234, "fr": 1230, "rl": 1235, "rr": 1232}
}
```

## 🔄 Roadmapa

### Po otrzymaniu API czujników:
1. [ ] Zaimplementuj odbieranie danych z VL53L7CX przez UART
2. [ ] Dodaj obsługę danych IMU ICM-20948
3. [ ] Zaimplementuj odczyt enkoderów kwadraturowych
4. [ ] Dodaj fuzję sensorów (kalman filter) dla pozycji

## ⚠️ Uwagi
- Ten kod jest przeznaczony do wgrania na **ESP32-S3 #2 (Brain)**.
- ESP32 #1 (Hardware Controller) musi być wcześniej skonfigurowany.
- Upewnij się że połączenie UART jest poprawne (GPIO16/17).
- Logika SLAM i algorytmy działają na **ESP32 #2**.
