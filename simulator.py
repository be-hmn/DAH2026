"""
simulator.py — 파랑팀 드론 MAVLink 시뮬레이터

- 기본: 고정 circle / patrol 패턴
- state.blue_orders[uid] 있으면: 지휘관이 내린 목표 좌표로 이동
- 목표 도착(~1km) 시 자동으로 명령 해제 → 패턴 복귀
"""
import math, time, threading
import state
from pymavlink import mavutil
from config import SYSID_MAP, PATHS, BLUE_SPEED_MULTIPLIER

ARRIVE_DEG = 0.009   # ~1 km — 목표 도착 판정


def _path_speed(p):
    """경로 설정에서 step당 최대 이동 거리(deg) — BLUE_SPEED_MULTIPLIER 배속 적용"""
    spd = p['spd'] * BLUE_SPEED_MULTIPLIER
    if p['type'] == 'circle':
        return p['r'] * spd
    else:
        dist = math.sqrt((p['p2'][0] - p['p1'][0])**2 + (p['p2'][1] - p['p1'][1])**2)
        return dist * spd / 2


def _run():
    conns = {}
    for sysid, uid in SYSID_MAP.items():
        try:
            c = mavutil.mavlink_connection('udpout:127.0.0.1:14550',
                                           source_system=sysid, source_component=1)
            conns[uid] = c
        except Exception as e:
            print(f'[SIM] {uid} 연결 실패: {e}')
    print(f'[SIM] MAVLink 시뮬레이터 시작 ({len(conns)}개 유닛)')

    # 드론별 실제 위치 추적 (명령 이동 시 필요)
    actual_pos = {}
    for uid, p in PATHS.items():
        if p['type'] == 'circle':
            actual_pos[uid] = (p['cx'] + p['r'] * math.sin(p['phase']),
                               p['cy'] + p['r'] * math.cos(p['phase']))
        else:
            actual_pos[uid] = (p['p1'][0], p['p1'][1])

    ordered_log = {uid: False for uid in PATHS}

    step = 0
    while True:
        step += 1

        with state.lock:
            orders = dict(state.blue_orders)

        for uid, conn in conns.items():
            p         = PATHS[uid]
            speed     = _path_speed(p)
            cur_lat, cur_lon = actual_pos[uid]

            ordered = False
            if uid in orders:
                o = orders[uid]
                dest_lat, dest_lon = o['lat'], o['lon']
                dlat = dest_lat - cur_lat
                dlon = dest_lon - cur_lon
                dist_deg = math.sqrt(dlat**2 + dlon**2)

                if dist_deg < ARRIVE_DEG:
                    # 도착 → 명령 해제
                    with state.lock:
                        state.blue_orders.pop(uid, None)
                    if ordered_log[uid]:
                        print(f'[SIM] {uid} 목표 도착 — 명령 완료, 패턴 복귀')
                        ordered_log[uid] = False
                else:
                    # 목표로 이동
                    move = min(speed, dist_deg)
                    lat  = cur_lat + (dlat / dist_deg) * move
                    lon  = cur_lon + (dlon / dist_deg) * move
                    ordered = True
                    if not ordered_log[uid]:
                        print(f'[SIM] {uid} 명령 이동 시작 → LAT {dest_lat:.5f} LON {dest_lon:.5f} ({o.get("mission","")})')
                        ordered_log[uid] = True

            if not ordered:
                # 고정 패턴
                if p['type'] == 'circle':
                    angle = p['phase'] + step * p['spd'] * BLUE_SPEED_MULTIPLIER
                    lat   = p['cx'] + p['r'] * math.sin(angle)
                    lon   = p['cy'] + p['r'] * math.cos(angle)
                else:
                    t   = (math.sin(step * p['spd'] * BLUE_SPEED_MULTIPLIER) + 1) / 2
                    lat = p['p1'][0] + (p['p2'][0] - p['p1'][0]) * t
                    lon = p['p1'][1] + (p['p2'][1] - p['p1'][1]) * t

            actual_pos[uid] = (lat, lon)

            try:
                conn.mav.global_position_int_send(
                    (step * 100) & 0xFFFFFFFF,
                    int(lat * 1e7), int(lon * 1e7),
                    150000, 150000, 0, 0, 0, 0
                )
            except Exception:
                pass

        time.sleep(0.5)  # 2Hz


def start():
    threading.Thread(target=_run, daemon=True).start()
