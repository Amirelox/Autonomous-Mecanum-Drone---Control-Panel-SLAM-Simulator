# 🚀 Przewodnik Uruchomienia - Rzeczywisty Robot

## 📋 Architektura Systemu

```
┌─────────────────────┐         WebSocket          ┌──────────────────┐
│   ESP32-S3-Zero     │ ◄────────────────────────► │  Raspberry Pi    │
│   (na robocie)      │      ws://ESP32_IP/ws      │                  │
│                     │       (port 8765)          │                  │
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

---

## 🔧 KROK 1: Przygotowanie ESP32-S3

### 1.1 Flash MicroPython

```bash
# Pobierz firmware MicroPython dla ESP32-S3
# https://micropython.org/download/esp32s3/

# Wyczyść flash
esptool.py --port COM3 erase_flash

# Wgraj firmware
esptool.py --port COM3 write_flash -z 0x1000 esp32s3-*.bin
```

### 1.2 Zainstaluj biblioteki

Połącz się z REPL ESP32 przez serial (Thonny IDE lub PuTTY):

```python
# Sprawdź dostępne moduły
import uasyncio
import ujson
print("OK")
```

### 1.3 Wgraj pliki na ESP32

Użyj Thonny IDE lub `ampy`:

```bash
# Zainstaluj ampy
pip install adafruit-ampy

# Wgraj pliki
ampy --port COM3 put micropython/main.py
ampy --port COM3 put micropython/secrets.py
```

### 1.4 Skonfiguruj WiFi i klucze

Edytuj `secrets.py` na ESP32:

```python
# secrets.py
ROBOT_HMAC_KEY = "TWÓJ_64_ZNAKOWY_KLUCZ_HEX"  # Z api.md lub od administratora
ROBOT_WS_URL = "ws://192.168.1.100/ws"        # IP Raspberry Pi w sieci WiFi
```

**UWAGA:** ESP32 musi być w tej samej sieci WiFi co Raspberry Pi!

---

## 🖥️ KROK 2: Przygotowanie Raspberry Pi

### 2.1 Instalacja zależności

```bash
# Aktualizuj system
sudo apt update && sudo apt upgrade -y

# Zainstaluj Python i pip
sudo apt install python3 python3-pip python3-venv -y

# Utwórz środowisko wirtualne
cd ~/Autonomous-Mecanum-Drone
python3 -m venv venv
source venv/bin/activate

# Zainstaluj wymagane pakiety
pip install nicegui websockets numpy opencv-python-headless
```

### 2.2 Konfiguracja WiFi

Upewnij się, że Raspberry Pi ma stały IP w sieci lokalnej:

```bash
# Sprawdź IP
hostname -I

# Lub ustaw statyczne IP w /etc/dhcpcd.conf
sudo nano /etc/dhcpcd.conf
```

Dodaj na końcu:
```
interface wlan0
static ip_address=192.168.1.100/24
static routers=192.168.1.1
static domain_name_servers=8.8.8.8
```

### 2.3 Skonfiguruj klucze autoryzacji

Edytuj `robot_config.py`:

```python
# robot_config.py
MODE = "real_robot"  # Zmień z "simulation" na "real_robot"

ESP32_IP = "192.168.1.101"  # IP ESP32 w sieci WiFi
ESP32_WS_PORT = 8765
ESP32_HMAC_KEY = "TEN_SAM_KLUCZ_CO_NA_ESP32"  # Musi być identyczny!
```

---

## ▶️ KROK 3: Uruchomienie Systemu

### 3.1 Na ESP32-S3

1. Podłącz ESP32 do zasilania
2. ESP32 automatycznie połączy się z WiFi
3. Sprawdz IP ESP32 w routerze lub przez serial monitor

### 3.2 Na Raspberry Pi

```bash
# Przejdź do folderu projektu
cd ~/Autonomous-Mecanum-Drone

# Aktywuj środowisko
source venv/bin/activate

# Uruchom system
python init.py
```

Powinieneś zobaczyć:
```
Launching ecosystem environment with Web HMI Dashboard (NiceGUI)...
[API] WebSocket core server is active on port 8765...
All active software nodes initialized successfully.
👉 Direct your browser window to: http://localhost:8080
```

### 3.3 Otwórz Dashboard

W przeglądarce na Raspberry Pi (lub innym komputerze w sieci):
```
http://192.168.1.100:8080
```

---

## 🎮 KROK 4: Sterowanie Robotem

### 4.1 Pierwsze uruchomienie

1. **Dashboard** pokaże status "DISARMED"
2. Kliknij **"START DFS RUN"** - wyśle komendy neutralne do uzbrojenia
3. Robot zacznie eksplorację (Faza 1)

### 4.2 Monitorowanie

Dashboard pokazuje:
- ✅ Mapę odwiedzonych komórek
- ✅ Pozycję robota (X, Y, heading)
- ✅ Fazę: EXPLORATION → FAST RUN
- ✅ Dane z czujników (gdy API będzie gotowe)

### 4.3 Zatrzymanie

- **"STOP / EMERGENCY"** - natychmiastowe zatrzymanie
- **"GENERATE NEW MAZE & RESET"** - reset pozycji

---

## 🔍 Rozwiązywanie Problemów

### Problem: ESP32 nie łączy się z WiFi

```python
# Sprawdź w serial monitorze ESP32:
import network
sta = network.WLAN(network.STA_IF)
print(sta.ifconfig())
print(sta.isconnected())
```

**Rozwiązanie:** Upewnij się, że SSID i hasło są poprawne w `secrets.py`.

### Problem: Autentykacja nie przechodzi

Sprawdź logi na Raspberry Pi:
```
[WS] Authentication failed
```

**Rozwiązanie:**
1. Upewnij się, że `ESP32_HMAC_KEY` jest identyczny na ESP32 i RPi
2. Klucz musi być 64 znaki hex (32 bajty)
3. Sprawdź czy ESP32 i RPi mają ten sam czas (NTP)

### Problem: Robot nie reaguje na komendy

Sprawdź telemetrię w dashboardzie:
- `armed: 0` → Wyślij komendy neutralne (kliknij START)
- `age_ms > 350` → Strumieniowanie za wolne (sprawdź COMMAND_RATE_HZ)
- `estop: 1` → Wyślij `{"cmd": "resume"}`

### Problem: Dashboard nie ładuje mapy

Sprawdź konsolę przeglądarki (F12):
```javascript
// Błąd fetch('/api/status')
```

**Rozwiązanie:** Upewnij się, że NiceGUI działa na porcie 8080 i firewall nie blokuje.

---

## 📊 Diagnostyka

### Sprawdzenie połączenia WebSocket

Na Raspberry Pi:
```bash
# Test WebSocket
python3 -c "
import websocket
ws = websocket.create_connection('ws://192.168.1.101:8765/ws')
print('Connected:', ws.recv())
ws.close()
"
```

### Sprawdzenie czujników

Gdy API czujników będzie gotowe:
```python
# W REPL ESP32
from main import SensorAPI
sensors = SensorAPI()
sensors.init()
print(sensors.read_lidar())
print(sensors.read_imu())
```

### Logi systemu

```bash
# Logi NiceGUI
tail -f ~/.nicegui/logs/*.log

# Logi ESP32 (przez serial)
minicom -D /dev/ttyUSB0 -b 115200
```

---

## 🔄 Aktualizacja Kodu

### Na ESP32

```bash
# Wgraj nowy kod
ampy --port COM3 put micropython/main.py

# Restart ESP32
import machine
machine.reset()
```

### Na Raspberry Pi

```bash
# Pull z Gita
git pull origin main

# Restart
pkill -f "python init.py"
python init.py
```

---

## 📝 Checklist Przed Startem

- [ ] ESP32 wgrany z MicroPython
- [ ] `secrets.py` skonfigurowany na ESP32
- [ ] ESP32 połączony z WiFi (ten sam network co RPi)
- [ ] `robot_config.py` ustawiony na `MODE = "real_robot"`
- [ ] Klucz HMAC identyczny na ESP32 i RPi
- [ ] Raspberry Pi ma stałe IP
- [ ] Wszystkie zależności zainstalowane (`pip install -r requirements.txt`)
- [ ] Firewall nie blokuje portów 8080 i 8765
- [ ] Czujniki podłączone do ESP32 (I2C/SPI)
- [ ] Silniki i sterownik sprawne

---

## 🆘 Kontakt i Wsparcie

- Dokumentacja API: `api.md`
- Lista części: `.txt`
- Kod symulacji (referencja): `client.py`, `dashboard.py`
- Kod MicroPython: `micropython/main.py`
