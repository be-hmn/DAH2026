import json, math, socket, time, threading
from config import RADAR_PORT, UNK_TTL
import state

MIN_DT = 0.3   # 2Hz(0.5s 간격) 프로토콜보다 짧은 dt는 수신 버스트 노이즈로 간주, 속도 재계산 생략

# radar.py 전용 수신 시각 — state.last_seen 는 drone.py 도 같은 uid('UNK-0')로 쓰기 때문에
# (TTL 판정용, 다른 목적) dt 계산에 그대로 쓰면 서로 타임스탬프를 덮어써 오염된다.
_last_rx_ts: dict = {}


def _kinematics(prev_lat, prev_lon, lat, lon, dt):
    """연속 수신값으로부터 속도/방향 산출 — 발신자가 직접 보고하는 값이 아니라 레이더가 추적으로 도출."""
    dlat    = lat - prev_lat
    dlon    = lon - prev_lon
    dist_km = math.sqrt((dlat * 111) ** 2 + (dlon * 111 * math.cos(math.radians(lat))) ** 2)
    speed_kmh = dist_km / dt * 3600
    heading   = math.degrees(math.atan2(dlon, dlat)) % 360
    return round(speed_kmh, 1), round(heading, 1)


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
            now = time.time()
            with state.lock:
                if uid not in state.units:
                    state.units[uid] = {'id': uid, 'type': 'UNK', 'lat': lat, 'lon': lon,
                                         'speed_kmh': 0.0, 'heading': 0.0}
                    print(f'[RADAR] 미식별 비행체 등록: {uid}')
                else:
                    prev = state.units[uid]
                    dt = now - _last_rx_ts.get(uid, now)
                    if dt >= MIN_DT:
                        speed_kmh, heading = _kinematics(prev['lat'], prev['lon'], lat, lon, dt)
                        prev['speed_kmh'], prev['heading'] = speed_kmh, heading
                    prev['lat'], prev['lon'] = lat, lon
                state.last_seen[uid] = now
            _last_rx_ts[uid] = now
        except socket.timeout:
            continue
        except Exception as e:
            print(f'[RADAR] 수신 오류: {e}')


def _reaper():
    while True:
        now = time.time()
        with state.lock:
            expired = [uid for uid, ts in state.last_seen.items()
                       if now - ts > UNK_TTL]
            for uid in expired:
                state.units.pop(uid, None)
                state.last_seen.pop(uid, None)
                _last_rx_ts.pop(uid, None)
                print(f'[RADAR] 미식별 비행체 제거 (신호 없음): {uid}')
        time.sleep(1.0)


def start():
    threading.Thread(target=_receive, daemon=True).start()
    threading.Thread(target=_reaper,  daemon=True).start()