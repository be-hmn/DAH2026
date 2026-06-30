import json, socket, time, threading
from config import RADAR_PORT, UNK_TTL
import state


def _receive():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', RADAR_PORT))
    sock.settimeout(1.0)
    print(f'[RADAR] 수신 대기 (UDP:{RADAR_PORT})')
    while True:
        try:
            data, _ = sock.recvfrom(1024)
            obj = json.loads(data.decode())
            uid = obj['id']
            lat = float(obj['lat'])
            lon = float(obj['lon'])
            with state.lock:
                if uid in state.spoofed_ids:
                    # attack_agent가 제어 중 — 실제 위치는 real_positions에만 기록
                    state.real_positions[uid] = {'lat': lat, 'lon': lon}
                    state.last_seen[uid] = time.time()
                    continue
                if uid not in state.units:
                    state.units[uid] = {'id': uid, 'type': 'UNK', 'lat': lat, 'lon': lon}
                    print(f'[RADAR] 미식별 비행체 등록: {uid}')
                else:
                    state.units[uid]['lat'] = lat
                    state.units[uid]['lon'] = lon
                state.last_seen[uid] = time.time()
        except socket.timeout:
            continue
        except Exception as e:
            print(f'[RADAR] 수신 오류: {e}')


def _reaper():
    while True:
        now = time.time()
        with state.lock:
            expired = [uid for uid, ts in state.last_seen.items()
                       if now - ts > UNK_TTL and uid not in state.spoofed_ids]
            for uid in expired:
                state.units.pop(uid, None)
                state.last_seen.pop(uid, None)
                print(f'[RADAR] 미식별 비행체 제거 (신호 없음): {uid}')
        time.sleep(1.0)


def start():
    threading.Thread(target=_receive, daemon=True).start()
    threading.Thread(target=_reaper,  daemon=True).start()