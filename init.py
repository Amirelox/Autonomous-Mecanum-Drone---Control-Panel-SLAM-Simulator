import subprocess
import sys
import time

print("Launching ecosystem environment with Web HMI Dashboard (NiceGUI)...")

# 1. Spawn the asynchronous physics simulation engine background process
server_process = subprocess.Popen([sys.executable, "main.py"])
time.sleep(2)  # Allow port 8765 time to bind successfully

# 2. Spawn the Web HMI Dashboard process via NiceGUI
dashboard_process = subprocess.Popen([sys.executable, "dashboard.py"])

print("All active software nodes initialized successfully.")
print("👉 Direct your browser window to: http://localhost:8080")

try:
    server_process.wait()
    dashboard_process.wait()
except KeyboardInterrupt:
    server_process.terminate()
    dashboard_process.terminate()