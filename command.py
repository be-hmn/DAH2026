"""
command.py — 파랑팀 지휘관 자연어 명령 → 드론 이동 지시 변환 (LLM)

흐름:
  지휘관 입력 → interpret() → Gemini → {'orders': [...]} → state.blue_orders
  simulator.py 가 state.blue_orders 를 보고 해당 드론 이동
"""
import copy, json, os, re, threading, time
import state

_lock  = threading.Lock()
_latest = {'text': None, 'command': None, 'orders': {}, 'ts': None}

FRIENDLY_IDS = {'ALPHA-1', 'BRAVO-2', 'CHARLIE-3', 'DELTA-4'}


def _parse_json(text: str):
    text = text.strip()
    m = re.search(r'```(?:json)?\s*([\s\S]*?)```', text)
    if m:
        text = m.group(1).strip()
    return json.loads(text)


def interpret(command_text: str) -> dict:
    """
    자연어 명령을 해석해 state.blue_orders 에 기록하고 결과 반환.
    반환: {'ok': True, 'reply': str, 'orders': dict} 또는 {'error': str}
    """
    api_key = os.environ.get('GEMINI_API_KEY')
    if not api_key:
        return {'error': 'GEMINI_API_KEY 미설정'}

    # 현재 전장 스냅샷
    with state.lock:
        units_snap  = list(copy.deepcopy(state.units).values())
        decoy_unit  = dict(state.units['UNK-0']) if 'UNK-0' in state.units else None
        cur_orders  = dict(state.blue_orders)

    friendly  = [u for u in units_snap if u['type'] != 'UNK']
    friendly_tx = '\n'.join(
        f"  {u['id']} ({u['type']}) LAT {u['lat']:.5f} LON {u['lon']:.5f}"
        + (f" [명령 진행 중 → {cur_orders[u['id']].get('mission','')}]" if u['id'] in cur_orders else '')
        for u in friendly
    )
    # 파랑팀은 디코이인지 모름 — 그냥 미식별 비행체로 표시
    decoy_tx = (
        f"  UNK-0 (미식별 비행체) LAT {decoy_unit['lat']:.5f} LON {decoy_unit['lon']:.5f}"
        if decoy_unit else '  없음'
    )

    # 최근 AI 어드바이저 분석
    try:
        import ai
        ai_text = ai.latest().get('text') or '없음'
    except Exception:
        ai_text = '없음'

    prompt = f"""당신은 파랑팀 전술 AI 보좌관입니다.
레이더에 미식별 비행체(UNK-0)가 포착되었습니다. 지휘관의 명령을 드론 이동 좌표로 변환하세요.

[아군 드론 현재 위치]
{friendly_tx}

[레이더 탐지 — 미식별 비행체]
{decoy_tx}

[AI 어드바이저 분석]
{ai_text}

[지휘관 명령]
{command_text}

응답 형식 (JSON만, 설명 없음):
{{
  "orders": [
    {{"id": "드론ID", "lat": 목표위도, "lon": 목표경도, "mission": "임무 한줄 설명"}}
  ],
  "reply": "지휘관에게 보내는 응답 (군사 어투, 한국어, 1~2줄)"
}}

규칙:
- 드론 ID는 ALPHA-1, BRAVO-2, CHARLIE-3, DELTA-4 중에서만
- 좌표는 서울/경기 권역 (LAT 37.3~37.9, LON 126.6~127.3) 기준
- 명령 없는 드론은 orders에 포함하지 않음
- CHARLIE-3 은 지상 유닛(GND), 이동 속도 느림
- JSON 외 출력 금지"""

    try:
        from google import genai
        import os as _os
        _saved = _os.environ.pop('GOOGLE_API_KEY', None)
        client = genai.Client(api_key=api_key)
        if _saved is not None:
            _os.environ['GOOGLE_API_KEY'] = _saved

        text = None
        for attempt in range(3):
            try:
                resp = client.models.generate_content(
                    model='gemini-2.5-flash-lite', contents=prompt
                )
                text = resp.text.strip()
                break
            except Exception as e:
                if '503' in str(e) or 'UNAVAILABLE' in str(e):
                    wait = 2 ** attempt
                    print(f'[CMD] 503 재시도 {attempt+1}/3 ({wait}s)')
                    time.sleep(wait)
                else:
                    raise

        if not text:
            return {'error': 'LLM 응답 없음'}

        parsed = _parse_json(text)
        raw_orders = parsed.get('orders', [])
        reply      = parsed.get('reply', '')

        # 유효 드론만 필터링
        orders = {
            o['id']: {
                'lat':     float(o['lat']),
                'lon':     float(o['lon']),
                'mission': o.get('mission', ''),
            }
            for o in raw_orders
            if o.get('id') in FRIENDLY_IDS
        }

        with state.lock:
            state.blue_orders.update(orders)

        with _lock:
            _latest['text']    = reply
            _latest['command'] = command_text
            _latest['orders']  = {k: dict(v) for k, v in orders.items()}
            _latest['ts']      = time.time()

        print(f'[CMD] 명령 해석: {command_text!r} → {len(orders)}개 드론 지시')
        for uid, o in orders.items():
            print(f'[CMD]   {uid} → LAT {o["lat"]:.5f} LON {o["lon"]:.5f}  ({o["mission"]})')

        return {'ok': True, 'reply': reply, 'orders': orders}

    except Exception as e:
        print(f'[CMD] 오류: {e}')
        return {'error': str(e)}


def clear_order(uid: str = None):
    """특정 드론(uid) 또는 전체 명령 해제"""
    with state.lock:
        if uid:
            state.blue_orders.pop(uid, None)
        else:
            state.blue_orders.clear()
    print(f'[CMD] 명령 해제: {uid or "전체"}')


def latest() -> dict:
    with _lock:
        return dict(_latest)
