"""
Konfiguracja połączenia z rzeczywistym robotem ESP32-S3
========================================================
Użyj tego pliku, gdy chcesz sterować prawdziwym robotem zamiast symulacji.
"""

# Tryb pracy: 'simulation' lub 'real_robot'
MODE = "simulation"  # Zmień na "real_robot" aby użyć ESP32

# Konfiguracja ESP32-S3
ESP32_IP = "192.168.1.100"  # IP ESP32 w sieci WiFi
ESP32_WS_PORT = 8765        # Port WebSocket na ESP32
ESP32_HMAC_KEY = ""         # Klucz autoryzacyjny (64 znaki hex)

# Timeouty
CONNECTION_TIMEOUT = 5.0    # Sekundy
WATCHDOG_TIMEOUT_MS = 350   # Milisekundy

# Częstotliwość komunikacji
COMMAND_RATE_HZ = 25        # Hz (wysyłanie komend do ESP32)
TELEMETRY_RATE_HZ = 10      # Hz (odbieranie danych z ESP32)
