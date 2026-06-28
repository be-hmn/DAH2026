import math, time, threading
from pymavlink import mavutil
from config import SYSID_MAP, PATHS


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

    step = 0
    while True:
        step += 1
        for uid, conn in conns.items():
            p = PATHS[uid]
            if p['type'] == 'circle':
                angle = p['phase'] + step * p['spd']
                lat   = p['cx'] + p['r'] * math.sin(angle)
                lon   = p['cy'] + p['r'] * math.cos(angle)
            else:
                t   = (math.sin(step * p['spd']) + 1) / 2
                lat = p['p1'][0] + (p['p2'][0] - p['p1'][0]) * t
                lon = p['p1'][1] + (p['p2'][1] - p['p1'][1]) * t
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