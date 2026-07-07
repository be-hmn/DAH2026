"""
defense_agent.py — 이상탐지 방어 에이전트 (블루팀 전용 LLM 노드, AI Advisor와 별개)

radar.py 가 연속 수신값으로부터 도출한 속도/방향과 위치 이력을 종합해 미식별
접촉(UNK)의 물리적 이상 여부를 LLM으로 판단한다. 전황을 요약하는 AI Advisor(ai.py)와
달리 GPS 스푸핑 탐지 하나에만 집중하는 별도 노드.

공격 측 AdaptiveController(attack_process.py)가 정확히 이 속도/방향 일관성을 실제
드론과 비슷하게 유지해 탐지를 피하려 드는 상대이므로, 두 LLM이 같은 축(물리적
자연스러움)을 두고 공방을 벌이는 구조.
"""
import json, os, re, threading, time
from collections import deque
import state
from drone import DRONE_HZ, DRONE_SPEED

HISTORY_LEN = 8     # uid별 보관 샘플 수
COOLDOWN    = 8.0   # 동일 uid 재판단 최소 간격 (초)
CHECK_EVERY = 2.0   # 감시 주기 (초)

# 정상 순항 속도 상한 — drone.py 의 배속 설정(DRONE_SPEED_MULTIPLIER)에서 직접 계산.
# 하드코딩된 실측 UAV 속도(예: 50~300km/h)를 쓰면 시뮬레이션 배속 때문에 정상 추적도
# 오탐(false positive)되므로, 반드시 drone.py 값을 따라가도록 유지한다.
MAX_SPEED_KMH = round(DRONE_SPEED * 111 * DRONE_HZ * 3600)

_lock       = threading.Lock()
_latest: dict    = {}   # uid -> {'verdict','reason','ts','speed_kmh','heading'}
_history: dict   = {}   # uid -> deque[(ts, speed_kmh, heading, lat, lon)]
_last_call: dict = {}   # uid -> 마지막 판단 시각


def _parse_json(text: str) -> dict:
    text = text.strip()
    text = re.sub(r'^```[a-z]*\n?', '', text)
    text = re.sub(r'\n?```$', '', text)
    return json.loads(text.strip())


def _gemini_client():
    key = os.environ.get('GEMINI_API_KEY')
    if not key:
        return None
    from google import genai
    saved = os.environ.pop('GOOGLE_API_KEY', None)
    client = genai.Client(api_key=key)
    if saved is not None:
        os.environ['GOOGLE_API_KEY'] = saved
    return client


def _call_gemini(uid: str, samples: list):
    client = _gemini_client()
    if client is None:
        print('[DEFENSE] GEMINI_API_KEY 미설정 — 판단 건너뜀')
        return

    t0   = samples[0][0]
    rows = '\n'.join(
        f"  t+{t - t0:5.1f}s  속도 {spd:6.1f}km/h  방향 {hdg:5.1f}°  LAT {lat:.5f} LON {lon:.5f}"
        for t, spd, hdg, lat, lon in samples
    )

    prompt = f"""당신은 GPS 스푸핑 탐지 AI입니다. 아래는 레이더가 추적 중인 미식별 접촉 {uid}의
최근 속도/방향/위치 이력입니다 (레이더가 연속 수신값으로부터 직접 산출한 값 — 발신자가
스스로 보고한 값이 아님).

주의: 이 시뮬레이션은 실제 배속(x3)이 적용되어 있어 정상 순항 속도가 실측 UAV보다 훨씬
높은 최대 약 {MAX_SPEED_KMH}km/h까지 나타난다. 이 값 이하는 절대 속도만으로 의심하지 말 것 —
{MAX_SPEED_KMH}km/h를 뚜렷이(대략 1.5배 이상) 초과하는 경우에만 속도 자체를 이상 신호로 본다.

{rows}

판단 기준:
- 실제 UAV는 속도·방향이 관성에 따라 점진적으로 변한다 (급격한 순간 가속/감속·방향 반전은 부자연스러움)
- 속도가 위 배속 기준({MAX_SPEED_KMH}km/h)의 1.5배를 크게 벗어나면 의심
- 위치가 매 샘플 지나치게 규칙적인 패턴으로만 변하는 것도 위조 신호일 수 있음

JSON만 출력 (마크다운 없음):
{{"verdict": "normal|suspect|spoofed", "reason": "판단 근거 한 문장"}}"""

    try:
        text = None
        for attempt in range(3):
            try:
                resp = client.models.generate_content(model='gemini-2.5-flash-lite', contents=prompt)
                text = resp.text
                break
            except Exception as e:
                if '503' in str(e) or 'UNAVAILABLE' in str(e):
                    wait = 2 ** attempt
                    print(f'[DEFENSE] 503 재시도 {attempt+1}/3 ({wait}s 대기)')
                    time.sleep(wait)
                else:
                    raise
        if not text:
            return

        data = _parse_json(text)
        _, spd, hdg, _, _ = samples[-1]
        with _lock:
            _latest[uid] = {
                'verdict':   data.get('verdict', 'normal'),
                'reason':    data.get('reason', ''),
                'ts':        time.time(),
                'speed_kmh': spd,
                'heading':   hdg,
            }
        print(f"[DEFENSE] {uid} 판단: {data.get('verdict')} — {data.get('reason', '')}")
    except Exception as e:
        print(f'[DEFENSE] 오류: {e}')


def _watcher():
    while True:
        time.sleep(CHECK_EVERY)
        with state.lock:
            unk = {uid: dict(u) for uid, u in state.units.items() if u.get('type') == 'UNK'}

        now = time.time()
        with _lock:
            for uid in list(_history.keys()):
                if uid not in unk:
                    _history.pop(uid, None)
                    _last_call.pop(uid, None)
                    _latest.pop(uid, None)

        for uid, u in unk.items():
            sample = (now, u.get('speed_kmh', 0.0), u.get('heading', 0.0), u['lat'], u['lon'])
            samples = None
            with _lock:
                hist = _history.setdefault(uid, deque(maxlen=HISTORY_LEN))
                hist.append(sample)
                if len(hist) >= 4 and now - _last_call.get(uid, 0.0) >= COOLDOWN:
                    _last_call[uid] = now
                    samples = list(hist)
            if samples:
                threading.Thread(target=_call_gemini, args=(uid, samples), daemon=True).start()


def latest() -> dict:
    with _lock:
        return {uid: dict(v) for uid, v in _latest.items()}


def start():
    threading.Thread(target=_watcher, daemon=True).start()
