UNITS_INIT = [
    {'id': 'ALPHA-1',   'type': 'UAV', 'lat': 37.5340, 'lon': 126.9850},
    {'id': 'BRAVO-2',   'type': 'UAV', 'lat': 37.5680, 'lon': 126.9280},
    {'id': 'CHARLIE-3', 'type': 'GND', 'lat': 37.5020, 'lon': 127.0430},
    {'id': 'DELTA-4',   'type': 'UAV', 'lat': 37.5910, 'lon': 127.0180},
]

SYSID_MAP = {1: 'ALPHA-1', 2: 'BRAVO-2', 3: 'CHARLIE-3', 4: 'DELTA-4'}

PATHS = {
    'ALPHA-1':   {'type': 'circle', 'cx': 37.5340, 'cy': 126.9850, 'r': 0.014, 'phase': 0.0,   'spd': 0.012},
    'BRAVO-2':   {'type': 'patrol', 'p1': [37.5680, 126.9100], 'p2': [37.5680, 126.9500],       'spd': 0.010},
    'CHARLIE-3': {'type': 'patrol', 'p1': [37.4900, 127.0380], 'p2': [37.5140, 127.0380],       'spd': 0.001},
    'DELTA-4':   {'type': 'circle', 'cx': 37.5910, 'cy': 127.0180, 'r': 0.020, 'phase': 2.094, 'spd': 0.010},
}

UNK_TTL    = 5.0
RADAR_PORT = 15550