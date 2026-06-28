#!/usr/bin/env python3
import logging
logging.getLogger('werkzeug').setLevel(logging.ERROR)

from flask import Flask, jsonify, render_template, request
import threading, time, copy, math, os

app = Flask(__name__)

UNITS_INIT = [
    {'id': 'ALPHA-1',   'type': 'UAV', 'lat': 37.5340, 'lon': 126.9850},
    {'id': 'BRAVO-2',   'type': 'UAV', 'lat': 37.5680, 'lon': 126.9280},
    {'id': 'CHARLIE-3', 'type': 'GND', 'lat': 37.5020, 'lon': 127.0430},
    {'id': 'DELTA-4',   'type': 'UAV', 'lat': 37.5910, 'lon': 127.0180},
]

SYSID_MAP = {1: 'ALPHA-1', 2: 'BRAVO-2', 3: 'CHARLIE-3', 4: 'DELTA-4'}

# circle: 원형 순환  /  patrol: 직선 왕복 (사인파)
PATHS = {
    'ALPHA-1':   {'type': 'circle', 'cx': 37.5340, 'cy': 126.9850, 'r': 0.014, 'phase': 0.0,   'spd': 0.012},  # UAV 고속
    'BRAVO-2':   {'type': 'patrol', 'p1': [37.5680, 126.9100], 'p2': [37.5680, 126.9500],       'spd': 0.010},  # UAV 고속
    'CHARLIE-3': {'type': 'patrol', 'p1': [37.4900, 127.0380], 'p2': [37.5140, 127.0380],       'spd': 0.001},  # GND 저속
    'DELTA-4':   {'type': 'circle', 'cx': 37.5910, 'cy': 127.0180, 'r': 0.020, 'phase': 2.094, 'spd': 0.010},  # UAV 고속
}

units     = {u['id']: dict(u) for u in UNITS_INIT}
last_seen = {}   # uid → timestamp, UNK 유닛 만료 추적
lock      = threading.Lock()

UNK_TTL = 5.0   # 초


def reaper():
    while True:
        now = time.time()
        with lock:
            expired = [uid for uid, ts in last_seen.items() if now - ts > UNK_TTL]
            for uid in expired:
                units.pop(uid, None)
                last_seen.pop(uid, None)
                print(f'[GCS] 미식별 비행체 제거 (신호 없음): {uid}')
        time.sleep(1.0)


def mavlink_simulator():
    from pymavlink import mavutil
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
                lat = p['cx'] + p['r'] * math.sin(angle)
                lon = p['cy'] + p['r'] * math.cos(angle)
            else:
                t = (math.sin(step * p['spd']) + 1) / 2
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


def mavlink_receiver():
    from pymavlink import mavutil
    try:
        conn = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
        print('[GCS] MAVLink 수신 대기 (UDP:14550)')
        while True:
            msg = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1.0)
            if msg is None:
                continue
            sysid = msg.get_srcSystem()
            uid   = SYSID_MAP.get(sysid, f'UNK-{sysid}')
            with lock:
                if uid not in units:
                    units[uid] = {'id': uid, 'type': 'UNK', 'lat': 0.0, 'lon': 0.0}
                    print(f'[GCS] 미식별 비행체 등록: sysid={sysid} → {uid}')
                units[uid]['lat'] = msg.lat / 1e7
                units[uid]['lon'] = msg.lon / 1e7
                last_seen[uid]    = time.time()
    except Exception as e:
        print(f'[GCS] MAVLink 수신 오류: {e}')


@app.route('/api/state')
def api_state():
    with lock:
        res = {'units': list(copy.deepcopy(units).values())}
    return jsonify(res)


@app.route('/api/move', methods=['POST'])
def api_move():
    d = request.get_json(force=True)
    uid, lat, lon = d.get('id'), d.get('lat'), d.get('lon')
    if uid and lat is not None and lon is not None:
        with lock:
            if uid in units:
                units[uid]['lat'] = lat
                units[uid]['lon'] = lon
                p = PATHS.get(uid, {})
                if p.get('type') == 'circle':
                    p['cx'], p['cy'] = lat, lon
                elif p.get('type') == 'patrol':
                    dlat = p['p2'][0] - p['p1'][0]
                    dlon = p['p2'][1] - p['p1'][1]
                    p['p1'] = [lat, lon]
                    p['p2'] = [lat + dlat, lon + dlon]
    return jsonify({'ok': True})


@app.route('/api/ai', methods=['POST'])
def api_ai():
    d = request.get_json(force=True)
    query = d.get('query', '현재 상황을 분석해줘.')

    with lock:
        current_units = list(copy.deepcopy(units).values())

    friendly = [u for u in current_units if u['type'] != 'UNK']
    unknown  = [u for u in current_units if u['type'] == 'UNK']

    warnings = []
    for unk in unknown:
        for frd in friendly:
            dist_km = math.sqrt((unk['lat']-frd['lat'])**2 + (unk['lon']-frd['lon'])**2) * 111
            if dist_km < 5:
                warnings.append(f"{unk['id']} ↔ {frd['id']} {dist_km:.1f}km")

    friendly_txt = '\n'.join(f"  {u['id']} ({u['type']}) LAT {u['lat']:.5f} LON {u['lon']:.5f}" for u in friendly)
    unknown_txt  = '\n'.join(f"  {u['id']} LAT {u['lat']:.5f} LON {u['lon']:.5f}" for u in unknown) or '  없음'
    warn_txt     = '\n'.join(f"  {w}" for w in warnings) or '  없음'

    prompt = f"""당신은 전술 지휘 보좌관 AI입니다. 실시간 전장 데이터를 기반으로 지휘관의 판단을 지원합니다.

[현재 전장 상태]
아군 유닛:
{friendly_txt}

미식별 비행체:
{unknown_txt}

근접 경보 (5km 이내):
{warn_txt}

[지휘관 질의]
{query}

지침: 간결한 군사 어투, 한국어 답변. 행동 권고 시 유닛 ID 명시. 3문장 이내."""

    api_key = os.environ.get('GEMINI_API_KEY')
    if not api_key:
        return jsonify({'error': 'GEMINI_API_KEY 환경변수 미설정'}), 500

    try:
        from google import genai
        client = genai.Client(api_key=api_key)
        resp = client.models.generate_content(model='gemini-2.0-flash', contents=prompt)
        return jsonify({'response': resp.text})
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/')
def index():
    return render_template('index.html')


if __name__ == "__main__":
    threading.Thread(target=mavlink_receiver,  daemon=True).start()
    threading.Thread(target=mavlink_simulator, daemon=True).start()
    threading.Thread(target=reaper,            daemon=True).start()
    print("[GCS] http://127.0.0.1:8080")
    print("[GCS] MAVLink UDP:14550")
    app.run(host="127.0.0.1", port=8080, threaded=True)
