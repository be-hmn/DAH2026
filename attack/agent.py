"""
attack/agent.py — GPS 스푸핑 공격 에이전트 (3-레이어 + 2 LLM 역할)

시나리오:
  주황 드론(UNK) = 우리 팀 드론. 적군 레이더가 실제 위치를 포착.
  공격 에이전트가 적군 GCS 디스플레이에 표시되는 좌표를 위조 → 적군 오판 유도.

Layer 1 (TargetProfile):     주황 드론 실제 위치 추적 — 속도벡터
Layer 2 (SpoofEngine):       적군 GCS 좌표 위조 — LLM 경로 웨이포인트 추종
Layer 3 (AdaptiveController): step_size 동적 조정 (물리적 자연스러움 유지)

LLM 역할 ①  RouteDesigner:    자연스러운 가짜 임무 경로(웨이포인트) 생성
LLM 역할 ②  ReactionAnalyzer: 파란 드론 행동 변화 감지 → 적군 반응 판단
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
DECOY_SPEED         = 0.00028 # decoy 독립 이동속도 (deg/step @2Hz ≈ 240 km/h)


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
# LLM 역할 ① — 실제 드론 우회 경로 설계 (DroneRouter)
# ══════════════════════════════════════════════════════════════════════════════
class DroneRouter:
    """Gemini — 실제 드론이 목표 좌표까지 파란 드론을 피해 이동하는 우회 경로 설계
    위조 위치(decoy)가 파란 드론을 유인 중인 상황을 활용해 안전한 경로 선택"""

    def __init__(self):
        self._client     = _gemini_client()
        self._lock       = threading.Lock()
        self._last_ts    = 0.0
        self.last_prompt = ''
        if not self._client:
            print('[ROUTER] GEMINI_API_KEY 미설정 — 직선 경로 사용')

    def request_route(self, real_lat: float, real_lon: float,
                      target_lat: float, target_lon: float,
                      spoof_lat: float, spoof_lon: float,
                      speed_kmh: float, blues: list[dict]) -> bool:
        """실제 드론 우회 경로 요청 — state.drone_waypoints 에 직접 기록"""
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

        dist_to_target = math.sqrt(
            ((target_lat - real_lat) * 111)**2 +
            ((target_lon - real_lon) * 111)**2
        )

        prompt = f"""당신은 UAV 전술 경로 설계 AI입니다.
우리 드론(주황)이 목표 좌표까지 적군 드론을 피해 안전하게 이동하는 우회 경로를 설계합니다.
현재 적군 GCS에는 위조 좌표(decoy)가 표시되어 적군 드론 일부가 그 위치로 유인되고 있습니다.

[현재 상황]
우리 드론 실제 위치 : LAT {real_lat:.5f}  LON {real_lon:.5f}
목표 좌표           : LAT {target_lat:.5f}  LON {target_lon:.5f}  (잔여 {dist_to_target:.2f} km)
위조 위치(decoy)    : LAT {spoof_lat:.5f}  LON {spoof_lon:.5f}
드론 속도           : {speed_kmh:.1f} km/h
작전 지역           : DMZ~서울 (LAT 37.48~38.00, LON 126.80~127.20)

[적군 드론 현재 위치]
{blues_txt}

[경로 설계 원칙]
1. 실제 위치 → 목표 좌표까지 안내하는 웨이포인트 5~8개 생성
2. 적군 드론과 최소 0.8km(0.007도) 이상 거리 유지
3. decoy가 적군 드론을 끌어당기고 있다면 반대 방향을 우선 활용
4. 목표에 가까울수록 더 직선에 가깝게 접근
5. 작전 지역 경계 이탈 금지 (LAT 37.48~38.00, LON 126.80~127.20)

경로 선택 이유를 reason 필드에 한 문장으로 기술하세요.
JSON만 출력 (마크다운 없음):
{{"reason": "ALPHA-1이 decoy 방향으로 이동 중이므로 동쪽 우회로 선택", "waypoints": [{{"lat": 37.55000, "lon": 126.97000}}, ...]}}"""

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
        """웨이포인트가 소진됐거나 아직 없으면 True"""
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
        with state.lock:
            if self.uid in state.units:
                state.units[self.uid]['lat'] = lat
                state.units[self.uid]['lon'] = lon
            else:
                state.units[self.uid] = {
                    'id': self.uid, 'type': 'UNK', 'lat': lat, 'lon': lon
                }
        self.inject_count += 1

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
        self.profile     = TargetProfile(target_id)
        self.engine      = SpoofEngine(target_id)
        self.router      = DroneRouter()
        self.controller  = AdaptiveController()
        self._running    = False
        self._success_n  = 0
        self._total_n    = 0
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

            # Layer 2: 위조 좌표(decoy) — 실제 드론에 가장 가까운 파란 드론 방향으로 유인
            if blues:
                nearest = min(blues, key=lambda b:
                    (b['lat'] - real_lat)**2 + (b['lon'] - real_lon)**2)
                s_lat, s_lon = self.engine.step_decoy(nearest['lat'], nearest['lon'])
            else:
                # 파란 드론 없으면 실제 heading +90° 방향으로 이탈
                s_lat, s_lon = self.engine.step(snap, waypoint=None)
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
            gap     = self.engine.gap_km(real_lat, real_lon)
            risk    = self.controller.last_result.get('risk', '—')
            wp_info = f'WP{self.route.wp_index}/{len(self.route.waypoints)}' if self.route.waypoints else 'NO_ROUTE'
            print(
                f'[ATTACK] #{self.engine.inject_count:04d} | '
                f'실제:({real_lat:.5f},{real_lon:.5f}) | '
                f'위조:({s_lat:.5f},{s_lon:.5f}) | '
                f'gap:{gap:.3f}km | {wp_info} | '
                f'성공률:{self._success_rate():.1f}% | risk:{risk}'
            )

            elapsed = time.time() - t0
            time.sleep(max(0.0, interval - elapsed))

    def _async_llm(self, snap, s_lat, s_lon, real_lat, real_lon, blues):
        # AdaptiveController
        result = self.controller.query(
            snap, s_lat, s_lon,
            self.engine.step_size, self.engine.inject_count, self._success_rate()
        )
        if 'step_multiplier' in result:
            old = self.engine.step_size
            self.engine.set_step_size(old * float(result['step_multiplier']))
            print(f'[ADAPT] risk={result.get("risk","?")} | '
                  f'step {old:.6f}→{self.engine.step_size:.6f} | {result.get("reason","")}')

        # DroneRouter — 실제 드론 우회 경로 갱신 (웨이포인트 소진 시)
        if self.router.needs_update():
            with state.lock:
                target = dict(state.drone_target)
            if target:
                threading.Thread(
                    target=self.router.request_route,
                    args=(real_lat, real_lon,
                          target['lat'], target['lon'],
                          s_lat, s_lon,
                          snap.get('speed_kmh', 100), blues),
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
        snap = self.profile.snapshot()
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
            'success_rate': round(self._success_rate(), 1),
            'llm_last':     self.controller.last_result,
            'profile':      snap,
            'target':       target,
            'route': {
                'waypoints': wps,
                'wp_index':  wp_idx,
                'total':     len(wps),
                'last_msg':  self.router.last_prompt,
            },
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
