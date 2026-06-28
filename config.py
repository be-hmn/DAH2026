UNITS_INIT = [
    {'id': 'ALPHA-1',   'type': 'UAV', 'lat': 37.5340, 'lon': 126.9850},
    {'id': 'BRAVO-2',   'type': 'UAV', 'lat': 37.5680, 'lon': 126.9280},
    {'id': 'CHARLIE-3', 'type': 'GND', 'lat': 37.5020, 'lon': 127.0430},
    {'id': 'DELTA-4',   'type': 'UAV', 'lat': 37.5910, 'lon': 127.0180},
]

SYSID_MAP = {1: 'ALPHA-1', 2: 'BRAVO-2', 3: 'CHARLIE-3', 4: 'DELTA-4'}

# spd 단위: rad/step (2Hz 기준)
# circle: v_kmh = r_km * spd * 2 * 3600  (r_km = r° * 111)
# patrol: avg_v_kmh = |Δ|_km * spd * (4/π) * 3600  (lon: 1°≈88km, lat: 1°≈111km)
# 목표 속도 — UAV 230 km/h (KUS-FS 250 km/h 기준), UGV 40 km/h
PATHS = {
    'ALPHA-1':   {'type': 'circle', 'cx': 37.5340, 'cy': 126.9850, 'r': 0.014, 'phase': 0.0,   'spd': 0.021},  # ~235 km/h
    'BRAVO-2':   {'type': 'patrol', 'p1': [37.5680, 126.9100], 'p2': [37.5680, 126.9500],       'spd': 0.028},  # ~226 km/h avg
    'CHARLIE-3': {'type': 'patrol', 'p1': [37.4900, 127.0380], 'p2': [37.5140, 127.0380],       'spd': 0.007},  # ~43 km/h avg
    'DELTA-4':   {'type': 'circle', 'cx': 37.5910, 'cy': 127.0180, 'r': 0.020, 'phase': 2.094, 'spd': 0.014},  # ~224 km/h
}

UNK_TTL    = 5.0
RADAR_PORT = 15550