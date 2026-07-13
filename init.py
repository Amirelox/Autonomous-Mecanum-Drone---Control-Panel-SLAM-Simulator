import subprocess
import sys
import time

print("Launching ecosystem environment with Dash HMI Dashboard...")

# 1. Spawn the asynchronous physics simulation engine background process
server_process = subprocess.Popen([sys.executable, "main.py"])
time.sleep(2) # Allow port 8765 time to bind successfully

# 2. Spawn the Dash HMI Dashboard process
dashboard_process = subprocess.Popen([sys.executable, "dashboard_dash.py"])

print("All active software nodes initialized successfully.")
print("👉 Direct your browser window to: http://localhost:8501")

try:
    server_process.wait()
    dashboard_process.wait()
except KeyboardInterrupt:
    server_process.terminate()
    dashboard_process.terminate()