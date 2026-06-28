import threading
from config import UNITS_INIT

units: dict     = {u['id']: dict(u) for u in UNITS_INIT}
last_seen: dict = {}   # UNK uid → timestamp (아군 제외)
lock            = threading.Lock()