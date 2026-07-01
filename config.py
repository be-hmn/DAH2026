UNITS_INIT = [
    {'id': 'ALPHA-1',   'type': 'UAV', 'lat': 37.5340, 'lon': 126.9850},
    {'id': 'BRAVO-2',   'type': 'UAV', 'lat': 37.5680, 'lon': 126.9280},
    {'id': 'CHARLIE-3', 'type': 'GND', 'lat': 37.5020, 'lon': 127.0430},
    {'id': 'DELTA-4',   'type': 'UAV', 'lat': 37.5910, 'lon': 127.0180},
]

SYSID_MAP = {1: 'ALPHA-1', 2: 'BRAVO-2', 3: 'CHARLIE-3', 4: 'DELTA-4'}

# spd 단위: rad/step (2Hz 기준, 기준 실측 속도 — BLUE_SPEED_MULTIPLIER 적용 전)
# circle: v_kmh = r_km * spd * 2 * 3600  (r_km = r° * 111)
# patrol: avg_v_kmh = |Δ|_km * spd * (4/π) * 3600  (lon: 1°≈88km, lat: 1°≈111km)
# 목표 속도 — UAV 230 km/h (KUS-FS 250 km/h 기준), UGV 40 km/h
PATHS = {
    'ALPHA-1':   {'type': 'circle', 'cx': 37.5340, 'cy': 126.9850, 'r': 0.014, 'phase': 0.0,   'spd': 0.021},  # ~235 km/h
    'BRAVO-2':   {'type': 'patrol', 'p1': [37.5680, 126.9100], 'p2': [37.5680, 126.9500],       'spd': 0.028},  # ~226 km/h avg
    'CHARLIE-3': {'type': 'patrol', 'p1': [37.4900, 127.0380], 'p2': [37.5140, 127.0380],       'spd': 0.007},  # ~43 km/h avg
    'DELTA-4':   {'type': 'circle', 'cx': 37.5910, 'cy': 127.0180, 'r': 0.020, 'phase': 2.094, 'spd': 0.014},  # ~224 km/h
}

# 파랑팀 배속 — 위 PATHS 기준 실측 속도에 곱해 실제 시뮬레이션 속도를 낸다 (공격자 드론 3배속보다 낮게 유지)
BLUE_SPEED_MULTIPLIER = 1.8

UNK_TTL    = 5.0
RADAR_PORT = 15550   # attack_process → GCS radar.py (스푸핑 좌표 주입, MITM 지점)

RADAR_CENTER   = {'lat': 37.5665, 'lon': 126.9780}   # 레이더 기준점 — 서울 중심(시청)
RADAR_RANGE_KM = 50.0                                 # 레이더 탐지 반경 — 이 안에 들어오면 상시 탐지·스푸핑

# attack_process.py 를 별도 프로세스로 분리하기 위한 UDP 채널
ATTACK_IN_PORT  = 15551   # GCS → attack_process (텔레메트리: 실제 위치/파랑팀 위치, 컨트롤 명령)
ATTACK_OUT_PORT = 15552   # attack_process → GCS (상태 조회용 status, 드론 우회 waypoints)