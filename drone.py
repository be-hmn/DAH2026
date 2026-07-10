"""
drone.py — 우리 팀 드론(주황, UNK-0) 인프로세스 시뮬레이터

- state.drone_target 방향으로 이동
- state.drone_waypoints (DroneRouter LLM이 설정) 가 있으면 그것을 따라 우회
- state.real_positions['UNK-0'] 에 직접 기록 (UDP/radar 불필요)
- unk_sim.py 는 더 이상 별도 실행하지 않아도 됨
"""
import math, random, threading, time
import state

UID           = 'UNK-0'
DRONE_HZ      = 2.0
DRONE_SPEED   = 0.00084     # deg/step @2Hz ≈ 720 km/h (3배속)
WP_ARRIVE_DEG = 0.0005      # ≈ 55 m 이내 → 웨이포인트 도착

# 출발 좌표 — DMZ 인근 (파주/연천 방면)
START_LAT = 37.9200
START_LON = 126.9500


def _loop():
    lat = START_LAT
    lon = START_LON

    # 초기 위치 등록
    with state.lock:
        state.real_positions[UID] = {'lat': round(lat, 7), 'lon': round(lon, 7)}

    interval = 1.0 / DRONE_HZ
    print(f'[DRONE] 초기 위치: LAT {lat:.5f} LON {lon:.5f} (DMZ 근처)')

    while True:
        t0 = time.time()

        with state.lock:
            wps    = list(state.drone_waypoints)
            wp_idx = state.drone_wp_index
            target = dict(state.drone_target) if state.drone_target else None

        # 현재 웨이포인트 도착 확인 → 다음으로
        if wps and wp_idx < len(wps):
            wp = wps[wp_idx]
            if math.sqrt((lat - wp[0])**2 + (lon - wp[1])**2) < WP_ARRIVE_DEG:
                with state.lock:
                    state.drone_wp_index += 1
                    wp_idx += 1

        # 목표 결정: 우회 웨이포인트 > 최종 목표
        dest = None
        if wps and wp_idx < len(wps):
            dest = wps[wp_idx]
        elif target:
            dest = (target['lat'], target['lon'])

        # 이동
        if dest:
            dlat = dest[0] - lat
            dlon = dest[1] - lon
            dist = math.sqrt(dlat**2 + dlon**2)
            if dist > 1e-9:
                move = min(DRONE_SPEED, dist)
                lat += (dlat / dist) * move
                lon += (dlon / dist) * move

        with state.lock:
            state.real_positions[UID] = {'lat': round(lat, 7), 'lon': round(lon, 7)}
            state.last_seen[UID] = time.time()

        elapsed = time.time() - t0
        time.sleep(max(0.0, interval - elapsed))


def start(target: dict = None):
    """
    target: {'lat': float, 'lon': float}  ← 드론이 도달해야 할 목표 좌표
    """
    if target:
        with state.lock:
            state.drone_target = dict(target)
    t = state.drone_target
    print(f'[DRONE] 시작 → 목표: LAT {t.get("lat","?")} LON {t.get("lon","?")}')
    threading.Thread(target=_loop, daemon=True).start()
