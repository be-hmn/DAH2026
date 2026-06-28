#!/usr/bin/env python3
"""
미식별 비행체 시뮬레이터
사용법: uv run unk_sim.py [대수]  (기본값: 1)
sysid는 50번대부터 자동 할당 (아군 1-4와 충돌 없음)
"""
import sys, time, random, threading
from pymavlink import mavutil

TARGET   = 'udpout:127.0.0.1:14550'
COUNT    = int(sys.argv[1]) if len(sys.argv) > 1 else 1
SYSID_BASE = 50
STEP_MAX = 0.0008
BOUNDS   = {'lat': (37.50, 37.61), 'lon': (126.93, 127.08)}


def run_unit(sysid):
    conn = mavutil.mavlink_connection(TARGET, source_system=sysid, source_component=1)
    rng  = random.Random(sysid)

    lat  = rng.uniform(37.51, 37.60)
    lon  = rng.uniform(126.94, 127.07)
    dlat = rng.uniform(-STEP_MAX, STEP_MAX)
    dlon = rng.uniform(-STEP_MAX, STEP_MAX)

    print(f'[UNK-{sysid}] 시작  LAT {lat:.5f}  LON {lon:.5f}')
    step = 0
    while True:
        step += 1
        dlat += random.uniform(-STEP_MAX * 0.3, STEP_MAX * 0.3)
        dlon += random.uniform(-STEP_MAX * 0.3, STEP_MAX * 0.3)
        dlat  = max(-STEP_MAX, min(STEP_MAX, dlat))
        dlon  = max(-STEP_MAX, min(STEP_MAX, dlon))
        lat  += dlat
        lon  += dlon

        if not BOUNDS['lat'][0] <= lat <= BOUNDS['lat'][1]:
            dlat *= -1
            lat   = max(BOUNDS['lat'][0], min(BOUNDS['lat'][1], lat))
        if not BOUNDS['lon'][0] <= lon <= BOUNDS['lon'][1]:
            dlon *= -1
            lon   = max(BOUNDS['lon'][0], min(BOUNDS['lon'][1], lon))

        conn.mav.global_position_int_send(
            (step * 500) & 0xFFFFFFFF,
            int(lat * 1e7), int(lon * 1e7),
            100000, 100000, 0, 0, 0, 0
        )
        time.sleep(0.5)  # 2Hz


threads = []
for i in range(COUNT):
    sysid = SYSID_BASE + i
    t = threading.Thread(target=run_unit, args=(sysid,), daemon=True)
    t.start()
    threads.append(t)

print(f'[UNK] {COUNT}대 시뮬레이션 시작 (sysid {SYSID_BASE}~{SYSID_BASE+COUNT-1})')
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print('[UNK] 종료')
