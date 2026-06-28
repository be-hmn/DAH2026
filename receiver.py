import threading
from pymavlink import mavutil
from config import SYSID_MAP
import state


def _receive():
    try:
        conn = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
        print('[MAVLink] 수신 대기 (UDP:14550)')
        while True:
            msg = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1.0)
            if msg is None:
                continue
            sysid = msg.get_srcSystem()
            uid   = SYSID_MAP.get(sysid)
            if uid is None:
                print(f'[MAVLink] 경고: 미등록 sysid={sysid} 수신 (무시)')
                continue
            with state.lock:
                state.units[uid]['lat'] = msg.lat / 1e7
                state.units[uid]['lon'] = msg.lon / 1e7
    except Exception as e:
        print(f'[MAVLink] 수신 오류: {e}')


def start():
    threading.Thread(target=_receive, daemon=True).start()