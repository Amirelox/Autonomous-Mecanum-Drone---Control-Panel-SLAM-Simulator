import logging

# Single shared logger for debug diagnostics
debug_log = logging.getLogger("debug")
debug_log.setLevel(logging.DEBUG)

# File handler — writes to robot_debug.log
_fh = logging.FileHandler("robot_debug.log", mode="w", encoding="utf-8")
_fh.setLevel(logging.DEBUG)
_fh.setFormatter(logging.Formatter("%(asctime)s [%(name)s] %(levelname)s: %(message)s"))
debug_log.addHandler(_fh)
