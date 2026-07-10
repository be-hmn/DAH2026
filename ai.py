import copy, math, os, time, threading
import state

PROXIMITY_KM   = 5.0
COOLDOWN       = 15.0   # 이벤트 연속 호출 방지 (초)
PERIODIC_EVERY = 30.0   # 이벤트 없어도 주기적 분석 간격 (초)

_latest       = {'text': None, 'event': None, 'ts': None}
_prev_unks    = set()
_prev_alerts  = set()
_last_call_ts = 0.0
_lock         = threading.Lock()


def _proximity_alerts(units):
    friendly = [u for u in units if u['type'] != 'UNK']
    unknown  = [u for u in units if u['type'] == 'UNK']
    alerts   = set()
    for unk in unknown:
        for frd in friendly:
            mid_lat = math.radians((unk['lat'] + frd['lat']) / 2)
            dlat = (unk['lat'] - frd['lat']) * 111
            dlon = (unk['lon'] - frd['lon']) * 111 * math.cos(mid_lat)
            if math.sqrt(dlat**2 + dlon**2) < PROXIMITY_KM:
                alerts.add((unk['id'], frd['id']))
    return alerts


def _call_gemini(event_desc, snapshot):
    global _last_call_ts
    api_key = os.environ.get('GEMINI_API_KEY')
    if not api_key:
        print('[AI] GEMINI_API_KEY 미설정 — 분석 건너뜀')
        return

    friendly    = [u for u in snapshot if u['type'] != 'UNK']
    unknown     = [u for u in snapshot if u['type'] == 'UNK']
    alerts      = _proximity_alerts(snapshot)
    friendly_tx = '\n'.join(f"  {u['id']} ({u['type']}) LAT {u['lat']:.5f} LON {u['lon']:.5f}" for u in friendly)
    unknown_tx  = '\n'.join(f"  {u['id']} LAT {u['lat']:.5f} LON {u['lon']:.5f}" for u in unknown) or '  없음'
    alert_tx    = '\n'.join(f"  {a[0]} ↔ {a[1]}" for a in alerts) or '  없음'

    prompt = f"""당신은 전술 지휘 보좌관 AI입니다.

[탐지 이벤트]
{event_desc}

[현재 전장 상태]
아군:
{friendly_tx}
외부 UAV:
{unknown_tx}
근접 경보 (5km 이내):
{alert_tx}

지침: 간결한 군사 어투, 한국어. 마크다운(**, *, #) 사용 금지. "판단: …\n권고: …" 형식으로 2줄 이내."""

    try:
        import os as _os
        from google import genai
        _saved = _os.environ.pop('GOOGLE_API_KEY', None)
        client = genai.Client(api_key=api_key)
        if _saved is not None:
            _os.environ['GOOGLE_API_KEY'] = _saved

        text = None
        for attempt in range(3):
            try:
                resp = client.models.generate_content(model='gemini-2.5-flash-lite', contents=prompt)
                text = resp.text.strip()
                break
            except Exception as e:
                if '503' in str(e) or 'UNAVAILABLE' in str(e):
                    wait = 2 ** attempt
                    print(f'[AI] 503 재시도 {attempt+1}/3 ({wait}s 대기)')
                    time.sleep(wait)
                else:
                    raise

        if text:
            with _lock:
                _latest['text']  = text
                _latest['event'] = event_desc
                _latest['ts']    = time.time()
                _last_call_ts    = time.time()
            print(f'[AI] 분석 완료: {event_desc}')
    except Exception as e:
        print(f'[AI] Gemini 오류: {e}')


def _watcher():
    global _prev_unks, _prev_alerts, _last_call_ts
    _last_periodic = 0.0
    while True:
        time.sleep(2.0)
        with state.lock:
            snapshot = list(copy.deepcopy(state.units).values())

        cur_unks   = {u['id'] for u in snapshot if u['type'] == 'UNK'}
        cur_alerts = _proximity_alerts(snapshot)

        events = []
        for uid in cur_unks - _prev_unks:
            events.append(f'신규 외부 UAV 탐지: {uid}')
        for uid in _prev_unks - cur_unks:
            events.append(f'외부 UAV 신호 소실: {uid}')
        for pair in cur_alerts - _prev_alerts:
            events.append(f'근접 경보 발생: {pair[0]} ↔ {pair[1]} (5km 이내)')
        for pair in _prev_alerts - cur_alerts:
            events.append(f'근접 경보 해제: {pair[0]} ↔ {pair[1]}')

        _prev_unks   = cur_unks
        _prev_alerts = cur_alerts

        now = time.time()
        if events and now - _last_call_ts >= COOLDOWN:
            desc = ' / '.join(events)
            threading.Thread(target=_call_gemini, args=(desc, snapshot), daemon=True).start()
        elif not events and now - _last_periodic >= PERIODIC_EVERY and now - _last_call_ts >= COOLDOWN:
            # 이벤트 없어도 주기적으로 전장 상황 분석
            _last_periodic = now
            unk_ids = ', '.join(cur_unks) if cur_unks else '없음'
            alert_cnt = len(cur_alerts)
            desc = f'정기 전장 분석 — 외부 UAV: {unk_ids} | 근접 경보: {alert_cnt}건'
            threading.Thread(target=_call_gemini, args=(desc, snapshot), daemon=True).start()


def latest():
    with _lock:
        return dict(_latest)


def start():
    threading.Thread(target=_watcher, daemon=True).start()