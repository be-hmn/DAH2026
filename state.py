import threading
from config import UNITS_INIT

units: dict            = {u['id']: dict(u) for u in UNITS_INIT}
last_seen: dict        = {}   # UNK uid → timestamp (아군 제외)
real_positions: dict   = {}   # 실제 침투 드론 위치 (drone.py가 직접 기록, attack_process 별도 프로세스로 전송됨)
drone_target: dict     = {}   # 실제 드론 최종 목표 좌표 {'lat': float, 'lon': float}
drone_waypoints: list  = []   # attack_process(DroneRouter LLM)가 UDP로 보낸 우회 경로 [(lat, lon), ...]
drone_wp_index: int    = 0    # 현재 추종 중인 웨이포인트 인덱스
blue_orders: dict      = {}   # 파랑팀 지휘 명령: uid → {'lat', 'lon', 'mission'}
attack_status: dict    = {'running': False}   # attack_process가 UDP로 보낸 최신 상태 (API 캐시)
mission_complete: bool = False   # 침투 드론(UNK-0)이 목표 지점에 도달했는지 여부
lock                   = threading.Lock()