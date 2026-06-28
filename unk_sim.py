#!/usr/bin/env python3
"""
레이더 시스템 시뮬레이터
사용법: uv run unk_sim.py [대수]  (기본값: 1)
실제 레이더가 GCS로 전달하는 UDP JSON 피드를 모사한다.
"""
import sys, time, random, threading, socket, json

TARGET_HOST = '127.0.0.1'
TARGET_PORT = 15550
COUNT       = int(sys.argv[1]) if len(sys.argv) > 1 else 1
# 0.00035 deg/step @2Hz → lat 최대 ~280 km/h, 평균 ~200 km/h (적대 UAV 기준)
STEP_MAX    = 0.00035
PERTURB     = 0.10   # 방향 교란 비율 (STEP_MAX 대비), 낮을수록 직진성 증가
BOUNDS      = {'lat': (37.50, 37.61), 'lon': (126.93, 127.08)}


def run_unit(idx):
    uid  = f'UNK-{idx}'
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rng  = random.Random(idx)

    lat  = rng.uniform(37.51, 37.60)
    lon  = rng.uniform(126.94, 127.07)
    dlat = rng.uniform(-STEP_MAX, STEP_MAX)
    dlon = rng.uniform(-STEP_MAX, STEP_MAX)

    print(f'[{uid}] 시작  LAT {lat:.5f}  LON {lon:.5f}')
    while True:
        dlat += rng.uniform(-STEP_MAX * PERTURB, STEP_MAX * PERTURB)
        dlon += rng.uniform(-STEP_MAX * PERTURB, STEP_MAX * PERTURB)
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

        pkt = json.dumps({'id': uid, 'lat': round(lat, 7), 'lon': round(lon, 7)}).encode()
        sock.sendto(pkt, (TARGET_HOST, TARGET_PORT))
        time.sleep(0.5)  # 2Hz


threads = []
for i in range(COUNT):
    t = threading.Thread(target=run_unit, args=(i,), daemon=True)
    t.start()
    threads.append(t)

print(f'[RADAR-SIM] {COUNT}대 시뮬레이션 시작 → UDP {TARGET_HOST}:{TARGET_PORT}')
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print('[RADAR-SIM] 종료')