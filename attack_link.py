"""
attack_link.py — GCS ↔ attack_process.py(별도 프로세스) UDP 연결

송신: 2Hz, 파랑팀 위치 + 실제 침투 드론(UNK-0) 위치 + 잔여 웨이포인트 수 → attack_process
수신: attack_process가 보낸 status(UI 캐시) / waypoints(드론 우회 경로) 를 state에 반영
"""
import json, socket, threading, time
import state
from config import ATTACK_IN_PORT, ATTACK_OUT_PORT

DRONE_UID = 'UNK-0'
_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def _send_telemetry():
    interval = 0.5   # 2Hz
    while True:
        with state.lock:
            blues = [dict(u) for u in state.units.values() if u.get('type') != 'UNK']
            drone = dict(state.real_positions.get(DRONE_UID, {}))
            wp_remaining     = max(0, len(state.drone_waypoints) - state.drone_wp_index)
            mission_complete = state.mission_complete
        if drone:
            pkt = {'type': 'telemetry', 'blues': blues, 'drone': drone,
                   'wp_remaining': wp_remaining, 'mission_complete': mission_complete}
            try:
                _sock.sendto(json.dumps(pkt).encode(), ('127.0.0.1', ATTACK_IN_PORT))
            except Exception as e:
                print(f'[ATTACK-LINK] 송신 오류: {e}')
        time.sleep(interval)


def _receive():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', ATTACK_OUT_PORT))
    sock.settimeout(1.0)
    print(f'[ATTACK-LINK] 수신 대기 (UDP:{ATTACK_OUT_PORT})')
    while True:
        try:
            data, _ = sock.recvfrom(65536)
            obj  = json.loads(data.decode())
            kind = obj.pop('type', None)
            if kind == 'status':
                with state.lock:
                    state.attack_status = obj
            elif kind == 'waypoints':
                pts = [(p[0], p[1]) for p in obj.get('points', [])]
                with state.lock:
                    state.drone_waypoints = pts
                    state.drone_wp_index  = 0
        except socket.timeout:
            continue
        except Exception as e:
            print(f'[ATTACK-LINK] 수신 오류: {e}')


def send_control(cmd: dict):
    """API → attack_process 컨트롤 명령 전송 (예: {'cmd':'set_target','target_id':'UNK-0'})"""
    pkt = {'type': 'control', **cmd}
    try:
        _sock.sendto(json.dumps(pkt).encode(), ('127.0.0.1', ATTACK_IN_PORT))
    except Exception as e:
        print(f'[ATTACK-LINK] 컨트롤 송신 오류: {e}')


def start():
    threading.Thread(target=_send_telemetry, daemon=True).start()
    threading.Thread(target=_receive, daemon=True).start()