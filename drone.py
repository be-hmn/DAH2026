"""
drone.py — 우리 팀 드론(주황, UNK-0) 인프로세스 시뮬레이터

- state.drone_target 방향으로 이동
- state.drone_waypoints (DroneRouter LLM이 설정) 가 있으면 그것을 따라 우회
- state.real_positions['UNK-0'] 에 직접 기록 (GCS 내부용)
- 매 틱 실제 좌표를 UDP로 RADAR_UPLINK_PORT 에도 송신한다 — 이것이 "진짜 레이더 신호"이며,
  GCS radar.py 는 이 포트를 듣지 않는다. attack_process.py 가 가로채 폐기하고
  조작된 좌표만 RADAR_PORT(GCS radar.py) 로 재주입한다 (가로채기→차단→조작→동일 포트 주입).
- unk_sim.py 는 더 이상 별도 실행하지 않아도 됨
"""
import json, math, os, random, socket, threading, time
import state
from config import RADAR_UPLINK_PORT

UID                    = 'UNK-0'
DRONE_HZ               = 2.0
DRONE_BASE_SPEED       = 0.00028   # deg/step @2Hz ≈ 240 km/h — 실측 기준 속도(배속 적용 전)
DRONE_SPEED_MULTIPLIER = 3.0       # 공격자 드론 배속 (수비 측 BLUE_SPEED_MULTIPLIER 1.8배보다 빠르게 유지)
DRONE_SPEED            = DRONE_BASE_SPEED * DRONE_SPEED_MULTIPLIER   # ≈ 720 km/h
WP_ARRIVE_DEG          = 0.0005    # ≈ 55 m 이내 → 웨이포인트 도착
TARGET_ARRIVE_DEG      = 0.001     # ≈ 110 m 이내 → 최종 목표 도달(임무 완료) 판정

# 출발 좌표 — DMZ 인근 (파주/연천 방면)
START_LAT = 37.9200
START_LON = 126.9500

_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def _emit_uplink(lat: float, lon: float):
    """진짜 레이더 신호 송신 — attack_process 가 없으면 그냥 유실된다 (수신자 없는 UDP)."""
    pkt = {'lat': round(lat, 7), 'lon': round(lon, 7)}
    try:
        _sock.sendto(json.dumps(pkt).encode(), ('127.0.0.1', RADAR_UPLINK_PORT))
    except OSError:
        pass


def _loop():
    lat = START_LAT
    lon = START_LON

    # 초기 위치 등록
    with state.lock:
        state.real_positions[UID] = {'lat': round(lat, 7), 'lon': round(lon, 7)}
    _emit_uplink(lat, lon)

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
        _emit_uplink(lat, lon)

        # 최종 목표 도달 판정 — 우회 웨이포인트가 모두 소진된 상태에서만
        final_leg = target and not (wps and wp_idx < len(wps))
        if final_leg:
            dist_to_target = math.sqrt(
                (lat - target['lat'])**2 + (lon - target['lon'])**2)
            if dist_to_target < TARGET_ARRIVE_DEG:
                with state.lock:
                    state.mission_complete = True
                print(f'[DRONE] ■ 목표 도달 — 임무 완료 (LAT {lat:.5f} LON {lon:.5f})')
                time.sleep(3)   # 프론트엔드가 완료 화면을 표시할 시간 확보
                print('[GCS] 임무 완료 — 프로그램 종료')
                os._exit(0)

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
