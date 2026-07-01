"""
attack/agent.py — GPS 스푸핑 공격 에이전트 (별도 프로세스로 구동, attack_process.py 참고)

시나리오:
  공격자 드론(UNK-0)이 대한민국 영공에서 목표 지점(롯데타워)으로 침투한다.
  레이더(서울 중심 반경 50km) 탐지 범위 안에 들어오면 그 위치는 정확히 탐지되지만,
  그 데이터가 GCS로 전달되는 경로에서 중간자 공격으로 좌표가 위조되어
  지휘관의 판단과 AI Advisor의 분석을 오판으로 유도한다.

  GCS 프로세스와는 공유 메모리(state.py)를 쓰지 않고 UDP로만 통신한다 (MITM 구조):
    GCS → 여기: 파랑팀 위치 + 실제 드론 위치 텔레메트리 (attack_process.py 가 수신)
    여기 → GCS: 위조 좌표(레이더 포트로 주입, radar.py 는 일반 접촉처럼 처리),
                드론 우회 웨이포인트, UI용 status

LLM 역할  DroneRouter:      파란 드론 위치 기반, 실제 드론이 물리적으로 발각되지 않을 우회 경로 계산
Layer     AdaptiveController: 위조 좌표 이동 step_size 물리적 자연스러움 유지
"""
import json, math, os, re, threading, time
from collections import deque
from typing import Callable, Optional

from config import RADAR_CENTER, RADAR_RANGE_KM


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


# ── 설정 ─────────────────────────────────────────────────────────────────────
INJECT_HZ           = 2.0    # 위조 패킷 주입 주파수
STEP_MIN_RATIO      = 0.20   # 하한: 실제 드론 한 스텝의 20%
STEP_MAX_RATIO      = 1.20   # 상한: 실제 드론 한 스텝의 120%

ADAPT_COOLDOWN      = 5.0    # AdaptiveController 호출 간격 (초)
ROUTE_COOLDOWN      = 30.0   # DroneRouter 호출 간격 (초, 웨이포인트 소진 시)
HISTORY_LEN         = 20     # 위치 이력 보관 개수


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
    """공격자 드론 실제 위치 추적 — 속도벡터"""

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
# LLM 역할 — 실제 드론 우회 경로 설계 (DroneRouter)
# ══════════════════════════════════════════════════════════════════════════════
class DroneRouter:
    """Gemini — 파란 드론 위치를 보고 실제 드론이 물리적으로 발각되지 않을 우회 경로 설계.

    설계된 웨이포인트는 send_waypoints 콜백으로 GCS(drone.py)에 UDP 전송한다
    (attack_process 는 별도 프로세스이므로 state.drone_waypoints 를 직접 못 씀)."""

    def __init__(self, send_waypoints: Callable[[list[tuple[float, float]]], None]):
        self._client       = _gemini_client()
        self._lock         = threading.Lock()
        self._last_ts      = 0.0
        self._send_waypoints = send_waypoints
        self.last_prompt   = ''
        self.route_total   = 0   # 최근 전송한 웨이포인트 개수 (status 표시용)
        if not self._client:
            print('[ROUTER] GEMINI_API_KEY 미설정 — 직선 경로 사용')

    def request_route(self, real_lat: float, real_lon: float,
                      target_lat: float, target_lon: float,
                      blues: list[dict]) -> bool:
        """실제 드론 우회 경로 — send_waypoints 콜백으로 GCS에 전송"""
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

        prompt = f"""당신은 UAV 생존 경로 설계 AI입니다.
공격자 드론이 파란 드론에게 물리적으로 발각되지 않고 목표 지점까지 침투하는 경로를 설계하세요.

[현재 상황]
공격자 드론 실제 위치 : LAT {real_lat:.5f}  LON {real_lon:.5f}
목표 좌표            : LAT {target_lat:.5f}  LON {target_lon:.5f}  (잔여 {dist_to_target:.2f} km)
작전 지역            : DMZ~서울 (LAT 37.48~38.00, LON 126.80~127.20)

[파란 드론 현재 위치]
{blues_txt}

[경로 설계 원칙]
1. 공격자 드론 실제 위치 → 목표 좌표까지 웨이포인트 5~8개
2. 파란 드론과 최소 5km 이상 거리 유지 (물리적 발각 방지)
3. 목표에 가까울수록 직선 접근
4. 작전 지역 경계 이탈 금지 (LAT 37.48~38.00, LON 126.80~127.20)

경로 선택 이유를 reason 필드에 한 문장으로 기술하세요.
JSON만 출력 (마크다운 없음):
{{"reason": "파란 드론 밀집 지역을 피해 동쪽 우회", "waypoints": [{{"lat": 37.85000, "lon": 127.05000}}, ...]}}"""

        try:
            text = _gemini_generate(self._client, prompt)
            data = _parse_json(text)
            wps  = [(w['lat'], w['lon']) for w in data.get('waypoints', [])]
            if len(wps) >= 2:
                reason = data.get('reason', '')
                self._send_waypoints(wps)
                with self._lock:
                    self.last_prompt = reason
                    self.route_total = len(wps)
                print(f'[ROUTER] {reason} | {len(wps)}개 WP: {wps[0]}→{wps[-1]}')
                return True
        except Exception as e:
            print(f'[ROUTER] 오류: {e}')
        return False


# ══════════════════════════════════════════════════════════════════════════════
# Layer 2 — 스푸핑 엔진 (룰 기반, 2Hz)
# ══════════════════════════════════════════════════════════════════════════════
class SpoofEngine:
    """GCS 좌표 위조 — 실제 헤딩 기준 90도 방향으로 점진적 이탈.

    위조 좌표는 send_decoy 콜백으로 GCS 레이더 포트에 UDP 주입한다 (radar.py 는
    일반 외부 접촉과 동일하게 처리 — MITM 지점이 여기)."""

    FALLBACK_ANGLE = 90.0   # 실제 헤딩 대비 이탈 각도

    def __init__(self, uid: str, send_decoy: Callable[[str, float, float], None]):
        self.uid          = uid
        self._send_decoy  = send_decoy
        self.spoof_lat: Optional[float] = None
        self.spoof_lon: Optional[float] = None
        self.step_size    = 0.00008
        self.inject_count = 0
        self.active       = False   # 레이더 탐지 범위 진입 시에만 True

    def init_spoof(self, lat: float, lon: float):
        if self.spoof_lat is None:
            self.spoof_lat = lat
            self.spoof_lon = lon
            print(f'[SPOOF] 초기화: ({lat:.6f}, {lon:.6f})')

    def _clamp_step(self, step_km: float) -> float:
        lo = max(step_km * STEP_MIN_RATIO, 0.000005) / 111
        hi = step_km * STEP_MAX_RATIO / 111
        return max(lo, min(self.step_size, hi))

    def step(self, profile: dict) -> tuple[float, float]:
        """실제 heading + 90도 방향으로 이탈 — 레이더 범위 내인 동안 상시 위조"""
        step_km = profile.get('step_km', 0.01)
        clamped = self._clamp_step(step_km)
        heading     = profile.get('heading', 0.0)
        dev_heading = math.radians((heading + self.FALLBACK_ANGLE) % 360)
        self.spoof_lat += clamped * math.cos(dev_heading)
        self.spoof_lon += clamped * math.sin(dev_heading)
        return self.spoof_lat, self.spoof_lon

    def inject(self, lat: float, lon: float):
        if not self.active:
            # 비활성 상태 — 주입 중단, GCS 쪽 TTL(radar.py)이 자연스럽게 접촉 만료시킴
            return
        self._send_decoy(self.uid, lat, lon)
        self.inject_count += 1

    def deactivate(self):
        self.active = False
        print('[SPOOF] 비활성화 — 레이더 탐지 범위 이탈')

    def set_step_size(self, s: float):
        self.step_size = max(0.000005, min(float(s), 0.002))

    def gap_km(self, real_lat: float, real_lon: float) -> float:
        if self.spoof_lat is None:
            return 0.0
        return math.sqrt(
            ((self.spoof_lat - real_lat) * 111) ** 2 +
            ((self.spoof_lon - real_lon) * 111) ** 2
        )


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
    """전체 파이프라인 — 레이더 범위 판정 + 위조 주입 + step 조정 + 우회 경로 설계.

    GCS 프로세스와 공유 메모리를 쓰지 않고 콜백/텔레메트리로만 통신한다:
      get_telemetry() → (real_lat, real_lon, blues, wp_remaining) | None
      send_decoy(uid, lat, lon)      → GCS 레이더 포트로 위조 좌표 UDP 주입
      send_waypoints(points)         → GCS(drone.py)로 우회 웨이포인트 UDP 전송
      send_status(status_dict)       → GCS로 UI용 상태 UDP 전송
      target                         → {'lat', 'lon'} 실제 드론 최종 목표 (고정)
    """

    def __init__(self, target_id: str,
                 get_telemetry: Callable[[], Optional[tuple]],
                 send_decoy: Callable[[str, float, float], None],
                 send_waypoints: Callable[[list[tuple[float, float]]], None],
                 send_status: Callable[[dict], None],
                 target: dict):
        self.target_id      = target_id
        self.get_telemetry  = get_telemetry
        self.send_status    = send_status
        self.target         = target
        self.profile        = TargetProfile(target_id)
        self.engine          = SpoofEngine(target_id, send_decoy)
        self.drone_router    = DroneRouter(send_waypoints)
        self.controller       = AdaptiveController()
        self._running        = False
        self._success_n       = 0
        self._total_n         = 0
        self._prev_spoof: Optional[tuple] = None
        self._wp_remaining    = 0   # 최근 텔레메트리에서 받은 실제 드론 잔여 웨이포인트 수

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

            # Layer 1: 실제 위치 + 파랑팀 위치 텔레메트리 (GCS에서 UDP 수신된 값)
            telem = self.get_telemetry()
            if telem is None:
                time.sleep(interval)
                continue
            real_lat, real_lon, blues, wp_remaining = telem
            self._wp_remaining = wp_remaining

            self.profile.update(real_lat, real_lon)

            if not self.profile.is_ready():
                time.sleep(interval)
                continue

            snap = self.profile.snapshot()
            self.engine.init_spoof(real_lat, real_lon)

            # Layer 2: 레이더 탐지 범위(서울 중심 반경) 진입 시 상시 위조
            dist_from_radar_km = math.sqrt(
                ((real_lat - RADAR_CENTER['lat']) * 111) ** 2 +
                ((real_lon - RADAR_CENTER['lon']) * 111 * math.cos(math.radians(real_lat))) ** 2
            )
            in_range = dist_from_radar_km <= RADAR_RANGE_KM

            if in_range:
                if not self.engine.active:
                    self.engine.spoof_lat = real_lat
                    self.engine.spoof_lon = real_lon
                    self.engine.active    = True
                    print(f'[SPOOF] 활성화 — 레이더 탐지 범위 진입 ({dist_from_radar_km:.1f}km)')
                s_lat, s_lon = self.engine.step(snap)
            else:
                if self.engine.active:
                    self.engine.deactivate()
                s_lat = self.engine.spoof_lat or real_lat
                s_lon = self.engine.spoof_lon or real_lon

            self.engine.inject(s_lat, s_lon)

            if self._prev_spoof:
                self._eval_success(real_lat, real_lon, *self._prev_spoof, s_lat, s_lon)
            self._prev_spoof = (s_lat, s_lon)

            # Layer 3 + LLM 비동기 호출
            threading.Thread(
                target=self._async_llm,
                args=(snap, s_lat, s_lon, real_lat, real_lon, blues),
                daemon=True,
            ).start()

            # 콘솔 출력 — 실제로 위조가 주입 중일 때만 (위조 근거가 있을 때만 로그)
            if self.engine.active:
                gap = self.engine.gap_km(real_lat, real_lon)
                print(
                    f'[ATTACK] #{self.engine.inject_count:04d} | '
                    f'실제:({real_lat:.5f},{real_lon:.5f}) | '
                    f'위조:({s_lat:.5f},{s_lon:.5f}) | '
                    f'레이더범위:{dist_from_radar_km:.1f}km | gap:{gap:.3f}km'
                )

            self.send_status(self.status(real_lat, real_lon, blues))

            elapsed = time.time() - t0
            time.sleep(max(0.0, interval - elapsed))

    def _async_llm(self, snap, s_lat, s_lon, real_lat, real_lon, blues):
        # AdaptiveController — 위조 step_size 조정
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

        # DroneRouter — 실제 드론 우회 경로 갱신 (웨이포인트 소진 시, 파란 드론 물리적 회피)
        target = self.target
        if self._wp_remaining <= 0 and target:
            threading.Thread(
                target=self.drone_router.request_route,
                args=(real_lat, real_lon, target['lat'], target['lon'], blues),
                daemon=True,
            ).start()

    def start(self):
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def stop(self):
        self._running = False
        print(f'[ATTACK] ■ {self.target_id} 스푸핑 중지')

    def status(self, real_lat: float = None, real_lon: float = None,
               blues: list[dict] = None) -> dict:
        real_lat = real_lat if real_lat is not None else self.profile.snapshot().get('lat', 0)
        real_lon = real_lon if real_lon is not None else self.profile.snapshot().get('lon', 0)
        blues    = blues if blues is not None else []
        snap = self.profile.snapshot()
        sl, sn = self.engine.spoof_lat or 0, self.engine.spoof_lon or 0

        def dist_km(alat, alon, blat, blon):
            return round(math.sqrt(((alat-blat)*111)**2 + ((alon-blon)*111)**2), 2)

        blue_dists = [
            {'id': b['id'],
             'dist_real_km':  dist_km(real_lat, real_lon, b['lat'], b['lon']),
             'dist_decoy_km': dist_km(sl, sn, b['lat'], b['lon']) if self.engine.active else None}
            for b in blues
        ]
        wp_index = max(0, self.drone_router.route_total - self._wp_remaining)
        return {
            'target_id':    self.target_id,
            'running':      self._running,
            'inject_count': self.engine.inject_count,
            'real_lat':     real_lat,
            'real_lon':     real_lon,
            'spoof_lat':    self.engine.spoof_lat,
            'spoof_lon':    self.engine.spoof_lon,
            'gap_km':       round(self.engine.gap_km(real_lat, real_lon), 3),
            'step_size':    self.engine.step_size,
            'decoy_active': self.engine.active,
            'success_rate': round(self._success_rate(), 1),
            'llm_last':     self.controller.last_result,
            'profile':      snap,
            'target':       self.target,
            'route': {
                'wp_index':  wp_index,
                'total':     self.drone_router.route_total,
                'last_msg':  self.drone_router.last_prompt,
            },
            'blue_dists': blue_dists,
        }


# ══════════════════════════════════════════════════════════════════════════════
# 싱글턴 인터페이스 — attack_process.py 가 configure() 로 IO 훅을 먼저 등록한다
# ══════════════════════════════════════════════════════════════════════════════
_agent: Optional[AttackAgent] = None
_io: dict = {}


def configure(get_telemetry: Callable[[], Optional[tuple]],
              send_decoy: Callable[[str, float, float], None],
              send_waypoints: Callable[[list[tuple[float, float]]], None],
              send_status: Callable[[dict], None],
              target: dict):
    _io.update(get_telemetry=get_telemetry, send_decoy=send_decoy,
               send_waypoints=send_waypoints, send_status=send_status, target=target)


def start(target_id: str = 'UNK-0'):
    global _agent
    if _agent and _agent._running:
        _agent.stop()
    _agent = AttackAgent(target_id, **_io)
    _agent.start()


def stop():
    if _agent:
        _agent.stop()


def status() -> dict:
    return _agent.status() if _agent else {'running': False}


def set_target(target_id: str):
    start(target_id)
