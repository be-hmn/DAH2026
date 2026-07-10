"""
attack/agent.py — 주황 드론 생존 에이전트

시나리오:
  주황 드론(UNK-0) = 우리 팀 드론. DMZ에서 출발해 목표 지점(롯데타워)까지 침투.
  파란 드론(적군)이 30km 이내 접근 시 GPS 위조(decoy)로 유인 → 실제 드론 경로 보호.

LLM 역할 ①  DecoyRouter:  decoy를 어디로 이동시켜야 파란 드론을 실제 드론 반대로 유인할지 계산
LLM 역할 ②  DroneRouter:  파란 드론 위치 + decoy 위치 기반으로 실제 드론 우회 경로 계산
Layer     AdaptiveController: decoy 이동 step_size 물리적 자연스러움 유지
"""
import json, math, os, re, threading, time
from collections import deque
from typing import Optional


def _parse_json(text: str) -> dict:
    """Gemini가 마크다운 코드블록으로 감쌀 경우도 처리"""
    text = text.strip()
    text = re.sub(r'^```[a-z]*\n?', '', text)
    text = re.sub(r'\n?```$', '', text)
    return json.loads(text.strip())


def _gemini_generate(client, prompt: str, retries: int = 3) -> str:
    """503 등 일시 오류 시 지수 백오프로 재시도"""
    for attempt in range(retries):
        try:
            resp = client.models.generate_content(
                model='gemini-2.5-flash-lite', contents=prompt)
            return resp.text
        except Exception as e:
            msg = str(e)
            if '503' in msg or 'UNAVAILABLE' in msg:
                wait = 2 ** attempt          # 1s → 2s → 4s
                print(f'[GEMINI] 503 재시도 {attempt+1}/{retries} ({wait}s 대기)')
                time.sleep(wait)
            else:
                raise
    raise RuntimeError(f'Gemini {retries}회 재시도 실패')

import state

# ── 설정 ─────────────────────────────────────────────────────────────────────
INJECT_HZ           = 2.0    # 위조 패킷 주입 주파수
STEP_MIN_RATIO      = 0.20   # 하한: 실제 드론 한 스텝의 20%
STEP_MAX_RATIO      = 1.20   # 상한: 실제 드론 한 스텝의 120%

ADAPT_COOLDOWN      = 5.0    # AdaptiveController 호출 간격 (초)
ROUTE_COOLDOWN      = 30.0   # RouteDesigner 호출 간격 (초, 웨이포인트 소진 시)
HISTORY_LEN         = 20     # 위치 이력 보관 개수
WAYPOINT_ARRIVE_DEG = 0.0003 # 웨이포인트 도착 판정 거리 (≈33m)
DECOY_SPEED         = 0.00084 # decoy 독립 이동속도 (deg/step @2Hz ≈ 720 km/h, 3배속)
DECOY_TRIGGER_KM    = 30.0   # 파란 드론이 이 거리 이내로 접근 시 decoy 활성화


def _gemini_client():
    key = os.environ.get('GEMINI_API_KEY')
    if not key:
        return None
    import os as _os
    from google import genai
    _saved = _os.environ.pop('GOOGLE_API_KEY', None)
    client = genai.Client(api_key=key)
    if _saved is not None:
        _os.environ['GOOGLE_API_KEY'] = _saved
    return client


# ══════════════════════════════════════════════════════════════════════════════
# Layer 1 — 정찰 (룰 기반, 2Hz)
# ══════════════════════════════════════════════════════════════════════════════
class TargetProfile:
    """주황 드론 실제 위치 추적 — 속도벡터"""

    def __init__(self, uid: str):
        self.uid      = uid
        self._history: deque = deque(maxlen=HISTORY_LEN)
        self._lock    = threading.Lock()

    def update(self, lat: float, lon: float):
        with self._lock:
            self._history.append((lat, lon, time.time()))

    def snapshot(self) -> dict:
        with self._lock:
            h = list(self._history)
        if len(h) < 2:
            return {}
        lat,   lon,   c_ts = h[-1]
        p_lat, p_lon, p_ts = h[-2]
        dt      = max(c_ts - p_ts, 1e-6)
        dlat    = lat - p_lat
        dlon    = lon - p_lon
        step_km = math.sqrt(
            (dlat * 111) ** 2 +
            (dlon * 111 * math.cos(math.radians(lat))) ** 2
        )
        return {
            'id':        self.uid,
            'lat':       lat,
            'lon':       lon,
            'dlat':      dlat,
            'dlon':      dlon,
            'step_km':   step_km,
            'speed_kmh': round(step_km / dt * 3600, 1),
            'heading':   round(math.degrees(math.atan2(dlon, dlat)) % 360, 1),
        }

    def is_ready(self) -> bool:
        with self._lock:
            return len(self._history) >= 2


# ══════════════════════════════════════════════════════════════════════════════
# LLM 역할 ① — Decoy 유인 경로 설계 (DecoyRouter)
# ══════════════════════════════════════════════════════════════════════════════
class DecoyRouter:
    """Gemini — 파란 드론을 실제 드론 반대 방향으로 유인할 decoy 웨이포인트 설계"""

    def __init__(self):
        self._client     = _gemini_client()
        self._lock       = threading.Lock()
        self._last_ts    = 0.0
        self.waypoints: list[tuple[float, float]] = []
        self.wp_index    = 0
        self.last_prompt = ''
        if not self._client:
            print('[DECOY] GEMINI_API_KEY 미설정 — 단순 이탈 방향 사용')

    def request_route(self, real_lat: float, real_lon: float,
                      spoof_lat: float, spoof_lon: float,
                      target_lat: float, target_lon: float,
                      blues: list[dict]) -> bool:
        """decoy 유인 경로 요청 — 파란 드론이 실제 드론 반대 방향으로 가도록"""
        if self._client is None:
            return False
        with self._lock:
            now = time.time()
            if now - self._last_ts < ROUTE_COOLDOWN:
                return False
            self._last_ts = now

        blues_txt = '\n'.join(
            f"  {b['id']} ({b['type']}) LAT {b['lat']:.5f} LON {b['lon']:.5f}"
            for b in blues
        ) or '  없음'

        prompt = f"""당신은 GPS 위조 유인 전술 AI입니다.
우리 드론(주황)이 목표까지 안전하게 이동할 수 있도록,
위조 드론(decoy)을 움직여 파란 드론들을 주황 드론의 반대 방향으로 유인하세요.

[현재 상황]
주황 드론 실제 위치 : LAT {real_lat:.5f}  LON {real_lon:.5f}
주황 드론 목표 좌표 : LAT {target_lat:.5f}  LON {target_lon:.5f}
decoy 현재 위치    : LAT {spoof_lat:.5f}  LON {spoof_lon:.5f}
작전 지역          : DMZ~서울 (LAT 37.48~38.00, LON 126.80~127.20)

[파란 드론 현재 위치]
{blues_txt}

[유인 전술 원칙]
1. decoy는 주황 드론의 실제 이동 경로와 반대 방향으로 이동해야 함
2. 파란 드론들이 decoy를 따라오도록 파란 드론 근처를 경유하되, 실제 드론과는 멀어져야 함
3. decoy 이동 속도는 실제 드론과 비슷하게 유지 (탐지 회피)
4. 웨이포인트 5~8개, 각 간격 0.003~0.010도
5. 작전 지역 경계 이탈 금지

전략 이유를 reason 필드에 한 문장으로 기술하세요.
JSON만 출력 (마크다운 없음):
{{"reason": "ALPHA-1을 북서쪽으로 유인해 주황 드론 남하 경로 확보", "waypoints": [{{"lat": 37.80000, "lon": 126.88000}}, ...]}}"""

        try:
            text = _gemini_generate(self._client, prompt)
            data = _parse_json(text)
            wps  = [(w['lat'], w['lon']) for w in data.get('waypoints', [])]
            if len(wps) >= 2:
                reason = data.get('reason', '')
                with self._lock:
                    self.waypoints   = wps
                    self.wp_index    = 0
                    self.last_prompt = reason
                print(f'[DECOY] {reason} | {len(wps)}개 WP: {wps[0]}→{wps[-1]}')
                return True
        except Exception as e:
            print(f'[DECOY] 오류: {e}')
        return False

    def current_target(self) -> Optional[tuple[float, float]]:
        with self._lock:
            if self.wp_index < len(self.waypoints):
                return self.waypoints[self.wp_index]
        return None

    def advance(self):
        with self._lock:
            self.wp_index += 1

    def exhausted(self) -> bool:
        with self._lock:
            return self.wp_index >= len(self.waypoints)


# ══════════════════════════════════════════════════════════════════════════════
# LLM 역할 ② — 실제 드론 우회 경로 설계 (DroneRouter)
# ══════════════════════════════════════════════════════════════════════════════
class DroneRouter:
    """Gemini — 파란 드론 위치 + decoy 위치를 보고 실제 드론 우회 경로 설계"""

    def __init__(self):
        self._client     = _gemini_client()
        self._lock       = threading.Lock()
        self._last_ts    = 0.0
        self.last_prompt = ''
        if not self._client:
            print('[ROUTER] GEMINI_API_KEY 미설정 — 직선 경로 사용')

    def request_route(self, real_lat: float, real_lon: float,
                      target_lat: float, target_lon: float,
                      spoof_lat: Optional[float], spoof_lon: Optional[float],
                      blues: list[dict]) -> bool:
        """실제 드론 우회 경로 — state.drone_waypoints 에 직접 기록"""
        if self._client is None:
            return False
        with self._lock:
            now = time.time()
            if now - self._last_ts < ROUTE_COOLDOWN:
                return False
            self._last_ts = now

        blues_txt = '\n'.join(
            f"  {b['id']} ({b['type']}) LAT {b['lat']:.5f} LON {b['lon']:.5f}"
            for b in blues
        ) or '  없음'

        decoy_txt = (f'LAT {spoof_lat:.5f}  LON {spoof_lon:.5f} (유인 중)'
                     if spoof_lat is not None else '비활성')

        dist_to_target = math.sqrt(
            ((target_lat - real_lat) * 111)**2 +
            ((target_lon - real_lon) * 111)**2
        )

        prompt = f"""당신은 UAV 생존 경로 설계 AI입니다.
주황 드론이 적군 드론에 탐지·추격당하지 않고 목표 지점까지 침투하는 경로를 설계하세요.
현재 위조 드론(decoy)이 적군 드론 일부를 반대 방향으로 유인하고 있습니다.

[현재 상황]
주황 드론 실제 위치 : LAT {real_lat:.5f}  LON {real_lon:.5f}
목표 좌표           : LAT {target_lat:.5f}  LON {target_lon:.5f}  (잔여 {dist_to_target:.2f} km)
decoy 위치         : {decoy_txt}
작전 지역           : DMZ~서울 (LAT 37.48~38.00, LON 126.80~127.20)

[파란 드론 현재 위치]
{blues_txt}

[경로 설계 원칙]
1. 주황 드론 실제 위치 → 목표 좌표까지 웨이포인트 5~8개
2. 파란 드론과 최소 5km 이상 거리 유지
3. decoy가 파란 드론을 유인 중이면 그 반대 방향(decoy쪽으로 파란 드론이 몰린 공백)을 활용
4. 목표에 가까울수록 직선 접근
5. 작전 지역 경계 이탈 금지 (LAT 37.48~38.00, LON 126.80~127.20)

경로 선택 이유를 reason 필드에 한 문장으로 기술하세요.
JSON만 출력 (마크다운 없음):
{{"reason": "decoy가 북서쪽으로 ALPHA-1 유인 중, 동쪽 우회로 안전", "waypoints": [{{"lat": 37.85000, "lon": 127.05000}}, ...]}}"""

        try:
            text = _gemini_generate(self._client, prompt)
            data = _parse_json(text)
            wps  = [(w['lat'], w['lon']) for w in data.get('waypoints', [])]
            if len(wps) >= 2:
                reason = data.get('reason', '')
                with state.lock:
                    state.drone_waypoints = wps
                    state.drone_wp_index  = 0
                with self._lock:
                    self.last_prompt = reason
                print(f'[ROUTER] {reason} | {len(wps)}개 WP: {wps[0]}→{wps[-1]}')
                return True
        except Exception as e:
            print(f'[ROUTER] 오류: {e}')
        return False

    def needs_update(self) -> bool:
        with state.lock:
            return state.drone_wp_index >= len(state.drone_waypoints)



# ══════════════════════════════════════════════════════════════════════════════
# Layer 2 — 스푸핑 엔진 (룰 기반, 2Hz)
# ══════════════════════════════════════════════════════════════════════════════
class SpoofEngine:
    """적군 GCS 좌표 위조 — RouteDesigner 웨이포인트 추종, 없으면 90도 이탈"""

    FALLBACK_ANGLE = 90.0   # 웨이포인트 없을 때 기본 이탈 방향

    def __init__(self, uid: str):
        self.uid          = uid
        self.spoof_lat: Optional[float] = None
        self.spoof_lon: Optional[float] = None
        self.step_size    = 0.00008
        self.inject_count = 0
        self.active       = False   # 파란 드론 접근 시에만 True

    def init_spoof(self, lat: float, lon: float):
        if self.spoof_lat is None:
            self.spoof_lat = lat
            self.spoof_lon = lon
            print(f'[SPOOF] 초기화: ({lat:.6f}, {lon:.6f})')

    def _clamp_step(self, step_km: float) -> float:
        lo = max(step_km * STEP_MIN_RATIO, 0.000005) / 111
        hi = step_km * STEP_MAX_RATIO / 111
        return max(lo, min(self.step_size, hi))

    def step(self, profile: dict,
             waypoint: Optional[tuple[float, float]] = None) -> tuple[float, float]:
        step_km = profile.get('step_km', 0.01)
        clamped = self._clamp_step(step_km)

        if waypoint:
            # 웨이포인트 방향으로 이동
            wp_lat, wp_lon = waypoint
            dlat = wp_lat - self.spoof_lat
            dlon = wp_lon - self.spoof_lon
            dist = math.sqrt(dlat**2 + dlon**2)
            if dist > 1e-9:
                move = min(clamped, dist)
                self.spoof_lat += (dlat / dist) * move
                self.spoof_lon += (dlon / dist) * move
        else:
            # 웨이포인트 없음 — 실제 heading + 90도 방향으로 이탈
            heading     = profile.get('heading', 0.0)
            dev_heading = math.radians((heading + self.FALLBACK_ANGLE) % 360)
            self.spoof_lat += clamped * math.cos(dev_heading)
            self.spoof_lon += clamped * math.sin(dev_heading)

        return self.spoof_lat, self.spoof_lon

    def inject(self, lat: float, lon: float):
        if not self.active:
            # 비활성 상태 — state.units에서 decoy 제거 (지도에서 사라짐)
            with state.lock:
                state.units.pop(self.uid, None)
            return
        with state.lock:
            if self.uid in state.units:
                state.units[self.uid]['lat'] = lat
                state.units[self.uid]['lon'] = lon
            else:
                state.units[self.uid] = {
                    'id': self.uid, 'type': 'UNK', 'lat': lat, 'lon': lon
                }
        self.inject_count += 1

    def deactivate(self):
        self.active = False
        with state.lock:
            state.units.pop(self.uid, None)
        print('[SPOOF] 비활성화 — 파란 드론 위협 없음')

    def set_step_size(self, s: float):
        self.step_size = max(0.000005, min(float(s), 0.002))

    def gap_km(self, real_lat: float, real_lon: float) -> float:
        if self.spoof_lat is None:
            return 0.0
        return math.sqrt(
            ((self.spoof_lat - real_lat) * 111) ** 2 +
            ((self.spoof_lon - real_lon) * 111) ** 2
        )

    def near_waypoint(self, wp: tuple[float, float]) -> bool:
        if self.spoof_lat is None:
            return False
        return math.sqrt(
            (self.spoof_lat - wp[0])**2 +
            (self.spoof_lon - wp[1])**2
        ) < WAYPOINT_ARRIVE_DEG

    def step_decoy(self, target_lat: float, target_lon: float) -> tuple[float, float]:
        """decoy 독립 이동 — 실제 드론 속도와 무관하게 target 방향으로 이동"""
        dlat = target_lat - self.spoof_lat
        dlon = target_lon - self.spoof_lon
        dist = math.sqrt(dlat**2 + dlon**2)
        if dist > 1e-9:
            move = min(DECOY_SPEED, dist)
            self.spoof_lat += (dlat / dist) * move
            self.spoof_lon += (dlon / dist) * move
        return self.spoof_lat, self.spoof_lon


# ══════════════════════════════════════════════════════════════════════════════
# Layer 3 — step_size 조정 (AdaptiveController)
# ══════════════════════════════════════════════════════════════════════════════
class AdaptiveController:
    """Gemini — 위조 스텝 속도가 물리적으로 자연스러운 범위인지 판단, step_size 조정"""

    def __init__(self):
        self._client   = _gemini_client()
        self._last_ts  = 0.0
        self._lock     = threading.Lock()
        self.last_result: dict = {}

    def query(self, profile: dict, spoof_lat: float, spoof_lon: float,
              step_size: float, inject_count: int, success_rate: float) -> dict:
        if self._client is None:
            return {}
        with self._lock:
            now = time.time()
            if now - self._last_ts < ADAPT_COOLDOWN:
                return {}
            self._last_ts = now

        real_lat      = profile.get('lat', 0)
        real_lon      = profile.get('lon', 0)
        gap_km        = math.sqrt(
            ((spoof_lat - real_lat) * 111) ** 2 +
            ((spoof_lon - real_lon) * 111) ** 2
        )
        step_km       = profile.get('step_km', 0.01)
        spoof_step_km = step_size * 111
        speed_ratio   = spoof_step_km / step_km if step_km > 0 else 1.0

        prompt = f"""당신은 GPS 스푸핑 step_size 조정 AI입니다.
위조 좌표의 이동 속도가 실제 드론 속도와 비슷해야 적군이 탐지 못합니다.
gap은 클수록 좋으며, 탐지 기준이 아닙니다.

현재 gap: {gap_km:.3f} km | 실제 스텝: {step_km*1000:.1f}m | 위조 스텝: {spoof_step_km*1000:.1f}m | 속도비: {speed_ratio:.2f}x

판단: 속도비 0.8 미만→1.3배 확대, 0.8~1.0→1.1배, 1.0~1.1→0.9배, 1.1초과→0.6배 축소

JSON만 출력: {{"risk": "safe", "step_multiplier": 1.1, "reason": "한 문장"}}"""

        try:
            text   = _gemini_generate(self._client, prompt)
            result = _parse_json(text)
            with self._lock:
                self.last_result = result
            return result
        except Exception as e:
            print(f'[ADAPT] 오류: {e}')
            return {}


# ══════════════════════════════════════════════════════════════════════════════
# 오케스트레이터
# ══════════════════════════════════════════════════════════════════════════════
class AttackAgent:
    """전체 파이프라인 — 경로 설계 + 위조 주입 + step 조정 + 적군 반응 분석"""

    def __init__(self, target_id: str):
        self.target_id   = target_id
        self.profile      = TargetProfile(target_id)
        self.engine       = SpoofEngine(target_id)
        self.decoy_router = DecoyRouter()
        self.drone_router = DroneRouter()
        self.controller   = AdaptiveController()
        self._running     = False
        self._success_n   = 0
        self._total_n     = 0
        self._prev_spoof: Optional[tuple] = None

    def _success_rate(self) -> float:
        return (self._success_n / self._total_n * 100) if self._total_n else 0.0

    def _eval_success(self, real_lat, real_lon, ps_lat, ps_lon, cs_lat, cs_lon):
        self._total_n += 1
        d_prev = math.sqrt(((ps_lat - real_lat)*111)**2 + ((ps_lon - real_lon)*111)**2)
        d_cur  = math.sqrt(((cs_lat - real_lat)*111)**2 + ((cs_lon - real_lon)*111)**2)
        if d_cur > d_prev:
            self._success_n += 1

    def _loop(self):
        print(f'[ATTACK] ▶ {self.target_id} 스푸핑 시작')
        interval = 1.0 / INJECT_HZ

        while self._running:
            t0 = time.time()

            # Layer 1: 실제 위치 + 파란 드론 위치 수집
            with state.lock:
                real  = state.real_positions.get(self.target_id) \
                     or state.units.get(self.target_id)
                blues = [dict(u) for u in state.units.values()
                         if u.get('type') != 'UNK']
            if real is None:
                time.sleep(interval)
                continue

            real_lat, real_lon = real['lat'], real['lon']
            self.profile.update(real_lat, real_lon)

            if not self.profile.is_ready():
                time.sleep(interval)
                continue

            snap = self.profile.snapshot()
            self.engine.init_spoof(real_lat, real_lon)

            # Layer 2: 위조 좌표(decoy) — 파란 드론 30km 이내 접근 시 활성화
            def _dist_km(b):
                return math.sqrt(
                    ((b['lat'] - real_lat) * 111) ** 2 +
                    ((b['lon'] - real_lon) * 111) ** 2
                )

            threatening = [b for b in blues if _dist_km(b) < DECOY_TRIGGER_KM]

            if threatening:
                if not self.engine.active:
                    # 최초 활성화 — decoy를 실제 위치에서 spawn
                    self.engine.spoof_lat = real_lat
                    self.engine.spoof_lon = real_lon
                    self.engine.active    = True
                    nearest = min(threatening, key=_dist_km)
                    print(f'[SPOOF] 활성화 — {nearest["id"]} 접근 ({_dist_km(nearest):.2f} km)')

                # DecoyRouter 웨이포인트 추종
                wp = self.decoy_router.current_target()
                if wp and self.engine.near_waypoint(wp):
                    self.decoy_router.advance()
                    wp = self.decoy_router.current_target()

                if wp:
                    s_lat, s_lon = self.engine.step_decoy(wp[0], wp[1])
                else:
                    # 웨이포인트 없음 — 실제 드론 반대 방향으로 단순 이탈
                    with state.lock:
                        tgt = dict(state.drone_target)
                    if tgt:
                        # 목표 방향의 반대로 이탈
                        heading_to_target = math.atan2(
                            tgt['lon'] - real_lon, tgt['lat'] - real_lat)
                        opp = heading_to_target + math.pi
                        self.engine.spoof_lat += DECOY_SPEED * math.cos(opp)
                        self.engine.spoof_lon += DECOY_SPEED * math.sin(opp)
                    s_lat = self.engine.spoof_lat
                    s_lon = self.engine.spoof_lon
            else:
                if self.engine.active:
                    self.engine.deactivate()
                s_lat = self.engine.spoof_lat or real_lat
                s_lon = self.engine.spoof_lon or real_lon

            self.engine.inject(s_lat, s_lon)

            if self._prev_spoof:
                self._eval_success(real_lat, real_lon, *self._prev_spoof, s_lat, s_lon)
            self._prev_spoof = (s_lat, s_lon)

            # Layer 3 + LLM ① ② 비동기 호출
            threading.Thread(
                target=self._async_llm,
                args=(snap, s_lat, s_lon, real_lat, real_lon, blues),
                daemon=True,
            ).start()

            # 콘솔 출력
            gap      = self.engine.gap_km(real_lat, real_lon)
            risk     = self.controller.last_result.get('risk', '—')
            near_txt = ', '.join(f'{b["id"]}:{_dist_km(b):.1f}km' for b in threatening) or '없음'
            decoy_wp = f'WP{self.decoy_router.wp_index}/{len(self.decoy_router.waypoints)}' \
                       if self.decoy_router.waypoints else 'NO_WP'
            print(
                f'[ATTACK] #{self.engine.inject_count:04d} | '
                f'실제:({real_lat:.5f},{real_lon:.5f}) | '
                f'위조:({s_lat:.5f},{s_lon:.5f}) | '
                f'decoy:{"ON" if self.engine.active else "OFF"} {decoy_wp} | '
                f'gap:{gap:.3f}km | 위협:{near_txt}'
            )

            elapsed = time.time() - t0
            time.sleep(max(0.0, interval - elapsed))

    def _async_llm(self, snap, s_lat, s_lon, real_lat, real_lon, blues):
        # AdaptiveController — decoy step_size 조정
        if self.engine.active:
            result = self.controller.query(
                snap, s_lat, s_lon,
                self.engine.step_size, self.engine.inject_count, self._success_rate()
            )
            if 'step_multiplier' in result:
                old = self.engine.step_size
                self.engine.set_step_size(old * float(result['step_multiplier']))
                print(f'[ADAPT] risk={result.get("risk","?")} | '
                      f'step {old:.6f}→{self.engine.step_size:.6f} | {result.get("reason","")}')

        with state.lock:
            target = dict(state.drone_target)

        # LLM ① DecoyRouter — decoy 유인 경로 갱신 (활성 + 웨이포인트 소진 시)
        if self.engine.active and self.decoy_router.exhausted() and target:
            threading.Thread(
                target=self.decoy_router.request_route,
                args=(real_lat, real_lon, s_lat, s_lon,
                      target['lat'], target['lon'], blues),
                daemon=True,
            ).start()

        # LLM ② DroneRouter — 실제 드론 우회 경로 갱신 (웨이포인트 소진 시)
        if self.drone_router.needs_update() and target:
            spoof_lat = self.engine.spoof_lat if self.engine.active else None
            spoof_lon = self.engine.spoof_lon if self.engine.active else None
            threading.Thread(
                target=self.drone_router.request_route,
                args=(real_lat, real_lon,
                      target['lat'], target['lon'],
                      spoof_lat, spoof_lon, blues),
                daemon=True,
            ).start()


    def start(self):
        self._running = True
        with state.lock:
            state.spoofed_ids.add(self.target_id)
        threading.Thread(target=self._loop, daemon=True).start()

    def stop(self):
        self._running = False
        with state.lock:
            state.spoofed_ids.discard(self.target_id)
            real = state.real_positions.get(self.target_id)
            if real and self.target_id in state.units:
                state.units[self.target_id]['lat'] = real['lat']
                state.units[self.target_id]['lon'] = real['lon']
        print(f'[ATTACK] ■ {self.target_id} 스푸핑 중지 — 좌표 복원')

    def status(self) -> dict:
        with state.lock:
            real   = state.real_positions.get(self.target_id) \
                  or state.units.get(self.target_id) or {}
            target = dict(state.drone_target)
            wps    = list(state.drone_waypoints)
            wp_idx = state.drone_wp_index
            blues  = [dict(u) for u in state.units.values() if u.get('type') != 'UNK']
        snap = self.profile.snapshot()
        rl, rn = real.get('lat', 0), real.get('lon', 0)
        sl, sn = self.engine.spoof_lat or 0, self.engine.spoof_lon or 0

        def dist_km(alat, alon, blat, blon):
            return round(math.sqrt(((alat-blat)*111)**2 + ((alon-blon)*111)**2), 2)

        blue_dists = [
            {'id': b['id'],
             'dist_real_km':  dist_km(rl, rn, b['lat'], b['lon']),
             'dist_decoy_km': dist_km(sl, sn, b['lat'], b['lon']) if self.engine.active else None}
            for b in blues
        ]
        return {
            'target_id':    self.target_id,
            'running':      self._running,
            'inject_count': self.engine.inject_count,
            'real_lat':     real.get('lat'),
            'real_lon':     real.get('lon'),
            'spoof_lat':    self.engine.spoof_lat,
            'spoof_lon':    self.engine.spoof_lon,
            'gap_km':       round(self.engine.gap_km(
                                real.get('lat', 0), real.get('lon', 0)), 3),
            'step_size':    self.engine.step_size,
            'decoy_active': self.engine.active,
            'success_rate': round(self._success_rate(), 1),
            'llm_last':     self.controller.last_result,
            'profile':      snap,
            'target':       target,
            'route': {
                'waypoints': wps,
                'wp_index':  wp_idx,
                'total':     len(wps),
                'last_msg':  self.drone_router.last_prompt,
            },
            'decoy_route': {
                'last_msg': self.decoy_router.last_prompt,
            },
            'blue_dists': blue_dists,
        }


# ══════════════════════════════════════════════════════════════════════════════
# 싱글턴 인터페이스
# ══════════════════════════════════════════════════════════════════════════════
_agent: Optional[AttackAgent] = None


def start(target_id: str = 'UNK-0'):
    global _agent
    if _agent and _agent._running:
        _agent.stop()
    _agent = AttackAgent(target_id)
    _agent.start()


def stop():
    if _agent:
        _agent.stop()


def status() -> dict:
    return _agent.status() if _agent else {'running': False}


def set_target(target_id: str):
    start(target_id)
