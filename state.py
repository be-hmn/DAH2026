import threading
from config import UNITS_INIT

units: dict            = {u['id']: dict(u) for u in UNITS_INIT}
last_seen: dict        = {}   # UNK uid → timestamp (아군 제외)
spoofed_ids: set       = set()  # attack_agent가 제어 중인 uid — radar.py가 무시
real_positions: dict   = {}   # spoofed uid의 실제 위치 (drone.py가 직접 기록)
drone_target: dict     = {}   # 실제 드론 최종 목표 좌표 {'lat': float, 'lon': float}
drone_waypoints: list  = []   # DroneRouter LLM이 설정한 우회 경로 [(lat, lon), ...]
drone_wp_index: int    = 0    # 현재 추종 중인 웨이포인트 인덱스
lock                   = threading.Lock()