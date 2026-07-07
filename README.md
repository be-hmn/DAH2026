# DAH — GCS Tactical Display

MAVLink 기반 지상통제시스템(GCS) 전술 디스플레이. Flask 백엔드와 Leaflet.js 기반 웹 UI로 아군(파랑팀) 유닛 위치를 실시간 추적하고, 레이더로 외부 UAV를 탐지하며, Gemini AI가 전장 변화를 자동 분석·지휘관 명령을 해석한다. 별도 프로세스로 구동되는 GPS 스푸핑 공격 에이전트가 레이더→GCS 전달 경로를 중간자 공격(MITM)으로 조작해 지휘관과 AI Advisor의 오판을 유도하는 시나리오를 포함한다.

## 아키텍처

```
[파랑팀 UAV × 4]  ── MAVLink (UDP:14550) ──▶  receiver.py  ──┐
                                                               │
[GCS 프로세스: gcs_system.py]                                 ├──▶ state.py ──▶ Flask API ──▶ 브라우저
  simulator.py  (파랑팀 MAVLink 송신, blue_orders 추종)       │
  drone.py      (침투 드론 UNK-0 실제 위치 시뮬레이션) ────┐  │
  radar.py      (UDP:15550 수신 → 외부 UAV 등록/만료,      │
                 연속 수신값으로 속도·방향 직접 산출) ──────┼──┘
  ai.py            (상태 감시 → Gemini 전황 분석)          │
  defense_agent.py (radar.py 산출 속도·방향 이력 종합       │
                     → Gemini GPS 스푸핑 이상탐지, ai.py와 별도 노드)
  command.py    (지휘관 자연어 명령 → Gemini → blue_orders) │
  attack_link.py(attack_process 와 UDP 연결)               │ UDP:15553 (진짜 좌표 — GCS radar.py 는
        │  ▲                                               │            이 포트를 듣지 않는다)
        │  │ UDP 텔레메트리(15551): 파랑팀 위치            ▼
        ▼  │ UDP(15552): status / 우회 waypoints    [attack_process.py — 독립 프로세스]
[attack_process.py]                                  가로채기(RADAR_UPLINK_PORT) → 원본 폐기(차단)
  GPS 스푸핑 에이전트 (LLM 2종 + 룰 기반 스푸핑 엔진)  → LLM으로 좌표 조작
        │
        ▼ UDP:15550 (조작된 좌표 재주입 — radar.py 는 일반 외부 접촉으로 처리, 원본과 동일 포트라 구분 불가)
   GCS radar.py
```

GCS와 attack_process는 메모리를 공유하지 않고 UDP JSON으로만 통신한다 — attack_process가 죽어도 GCS는 정상 동작(스푸핑만 비활성)하며, GCS 재시작 중에도 attack_process는 독립적으로 유지된다. 침투 드론의 진짜 좌표는 attack_link 텔레메트리로 흘러가지 않고 drone.py가 직접 UDP:15553으로 송신하며, attack_process가 그 포트를 가로채 소비한다(원본은 절대 GCS로 전달되지 않음).

## 모듈 구성

| 파일 | 역할 |
|---|---|
| `gcs_system.py` | 진입점 — Flask 앱 + GCS 쪽 전체 스레드 시작 |
| `config.py` | 모든 상수 (유닛 정보, 경로/속도, TTL, 포트) |
| `state.py` | GCS 공유 상태 (`units`, `blue_orders`, `attack_status`, `Lock` 등) |
| `simulator.py` | 파랑팀 MAVLink 송신 시뮬레이터 (2Hz), `blue_orders` 있으면 지휘 명령 우선 추종 |
| `receiver.py` | MAVLink 수신 → 파랑팀 위치 갱신 |
| `radar.py` | 레이더 포트(UDP:15550) 수신 → 외부 UAV 등록/만료, 연속 수신값으로 속도·방향 산출 |
| `drone.py` | 침투 드론(UNK-0) 실제 위치 시뮬레이션 — DMZ→목표(롯데타워), 우회 웨이포인트 추종 |
| `ai.py` | 상태 변화 감지 → Gemini 전술 분석 (판단/권고) |
| `defense_agent.py` | radar.py가 산출한 속도/방향/위치 이력 종합 → Gemini GPS 스푸핑 이상탐지 (ai.py와 별개 노드) |
| `command.py` | 파랑팀 지휘관 자연어 명령 → Gemini 해석 → `blue_orders` |
| `attack_link.py` | GCS ↔ attack_process 간 UDP 연결 (파랑팀 텔레메트리 송신, status/waypoints 수신) |
| `api.py` | Flask Blueprint (`/api/state`, `/api/move`, `/api/ai/*`, `/api/defense/*`, `/api/attack/*`, `/api/command/*`) |
| `attack_process.py` | 공격 에이전트 독립 실행 프로세스 — 가로채기(UDP:15553) + GPS 스푸핑 로직(DroneRouter LLM + 스푸핑 엔진) + GCS로 조작 좌표 재주입(UDP:15550), 전부 이 파일 하나 |
| `unk_sim.py` | 범용 레이더 피드 시뮬레이터 (UDP JSON, 임의 외부 접촉 테스트용) |
| `templates/index.html` | 전술 지도 UI (Leaflet, 위성영상, KRDS 다크 테마) |

## 파랑팀 유닛

| ID | 유형 | 경로 | 실측 기준 속도 | 배속 적용 후 |
|---|---|---|---|---|
| ALPHA-1 | UAV | 원형 순환 | ~235 km/h | ~423 km/h |
| BRAVO-2 | UAV | 직선 왕복 | ~226 km/h avg | ~407 km/h avg |
| CHARLIE-3 | GND | 직선 왕복 | ~43 km/h avg | ~77 km/h avg |
| DELTA-4 | UAV | 원형 순환 | ~224 km/h | ~403 km/h |

UAV 속도는 KUS-FS(한국군 전술 MALE UAV, 순항 250 km/h) 기준, GND는 전술 UGV 야지 기준으로 산정한 실측값에 `BLUE_SPEED_MULTIPLIER`(1.8배)를 곱해 실제 시뮬레이션 속도를 낸다. 공격자 드론(UNK-0)은 실측 기준 ~240 km/h에 `DRONE_SPEED_MULTIPLIER`(3배)를 적용해 ~720 km/h로, 파랑팀보다 항상 빠르게 유지된다. 지휘관 명령(`blue_orders`)이 있으면 목표 도착(~1km) 시까지 이 패턴을 벗어나 이동하고, 도착 후 자동 복귀한다.

## 설치 및 실행

```bash
# 의존성 설치
uv sync

# Gemini API 키 설정
echo "GEMINI_API_KEY=your_key_here" > .env

# 1) GCS 서버 실행 (먼저 띄운다)
uv run --env-file .env gcs_system.py
# → http://127.0.0.1:8080
# → MAVLink UDP:14550 수신 대기
# → 레이더 UDP:15550 수신 대기

# 2) GPS 스푸핑 공격 에이전트 — 별도 터미널, 별도 프로세스
#    GCS 기동 후 2~3초 뒤 실행 권장 (미실행 시 스푸핑만 비활성, GCS 자체는 정상 동작)
uv run --env-file .env attack_process.py
# → UDP:15553 에서 drone.py의 진짜 좌표를 가로채 소비
# → UDP:15550(레이더 포트)으로 조작된 좌표만 GCS에 재주입
```

Defense Agent 유무에 따른 공방전 비교 실험(같은 attack_process로 두 번 재현):

```bash
# Defense Agent 있음 (기본값)
uv run --env-file .env gcs_system.py

# Defense Agent 없음 — 공격 에이전트만 단독 동작
DEFENSE_AGENT_ENABLED=0 uv run --env-file .env gcs_system.py
```

attack_process는 GCS와 완전히 독립된 프로세스라 이 값을 몰라도 동일하게 스푸핑하므로, Defense Agent 유무만 격리해서 비교할 수 있다.

브라우저에서 `http://127.0.0.1:8080` 접속 후 확인할 것:
- 우측 사이드바 **Unidentified** 카드 — UNK-0 등장, LAT/LON/SPD/HDG 값이 계속 갱신됨
- **AI Advisor** 패널 — 신규 접촉·근접 경보 시 자동 분석 (cyan)
- **Defense Agent** 패널 — 2초 주기로 속도·방향 이상 여부 판단, `NORMAL`/`SUSPECT`/`SPOOFED` (red)
- 지도 위 UNK-0 마커가 실제 드론과 다른 경로로 표시되면 스푸핑이 GCS에 반영되고 있다는 뜻

외부 UAV 시뮬레이션 (선택, UNK-0 외의 임의 접촉 테스트용):

```bash
# N대 동시 시뮬레이션 (기본값: 1)
uv run unk_sim.py [N]
```

## REST API

| 엔드포인트 | 메서드 | 설명 |
|---|---|---|
| `/api/state` | GET | 전체 유닛 위치 반환 |
| `/api/move` | POST | 유닛 위치 수동 이동 |
| `/api/ai/latest` | GET | 최신 AI 분석 결과 반환 |
| `/api/defense/latest` | GET | Defense Agent의 uid별 이상탐지 판단(`verdict`/`reason`/`speed_kmh`/`heading`) 반환 |
| `/api/attack/status` | GET | 공격 에이전트 최신 상태 (attack_process가 UDP로 보낸 캐시) |
| `/api/attack/target` | POST | 공격 에이전트 보호 대상(target_id) 변경 |
| `/api/command` | POST | 지휘관 자연어 명령 해석 (동기, 1~3초 소요) |
| `/api/command/latest` | GET | 최근 명령 해석 결과 + 진행 중인 `blue_orders` |
| `/api/command/clear` | POST | 지휘 명령 해제 (`id` 없으면 전체 해제) |

`/api/move` 요청 형식:
```json
{ "id": "ALPHA-1", "lat": 37.534, "lon": 126.985 }
```

`/api/command` 요청 형식:
```json
{ "text": "ALPHA-1을 미확인 비행체 쪽으로 접근시켜" }
```

## AI Advisor

상태 감시 스레드(`ai.py`)가 2초 주기로 변화를 감지하고 Gemini에 자동 요청한다. 연속 호출 방지를 위해 15초 cooldown 적용.

**트리거 조건**
- 신규 외부 UAV 탐지
- 외부 UAV 신호 소실
- 아군↔외부 UAV 근접 경보 발생 (5km 이내)
- 근접 경보 해제

**응답 형식** — `판단: … / 권고: …` (한국어, 마크다운 없음)

사용 모델: `gemini-2.5-flash-lite`

## Defense Agent

`radar.py`가 연속 수신값에서 직접 산출한 속도·방향(발신자가 스스로 보고하는 값이 아님)을 종합해, 레이더 접촉(UNK)별 물리적 이상 여부를 판단하는 전용 감시 노드(`defense_agent.py`). AI Advisor와는 별개 스레드 — 전황 요약이 아니라 GPS 스푸핑 탐지 하나에만 집중한다.

**감시 주기**: 2초마다 UNK 접촉별 최근 8개 샘플을 모아, 접촉당 8초 쿨다운으로 Gemini에 판단 요청.

**판정 기준**
- 속도·방향이 관성에 따라 점진적으로 변하는지 (급격한 순간 가속/감속·방향 반전은 부자연스러움)
- 속도가 정상 순항 속도 상한(`drone.py`의 `DRONE_SPEED`에서 배속 반영해 동적 계산, 기본 ≈671km/h)의 1.5배를 크게 초과하는지
- 위치 변화가 매 샘플 지나치게 규칙적인 패턴을 보이는지

**판정 결과**: `normal` / `suspect` / `spoofed` 3단계 + 근거 한 문장, `/api/defense/latest`로 노출. UI 우측 하단 패널에 실시간 표시.

공격 측 `AdaptiveController`(attack_process.py)가 정확히 이 속도·방향 일관성을 실제 드론과 비슷하게 유지해 탐지를 피하려 드는 상대이므로, 두 LLM이 같은 축(물리적 자연스러움)을 두고 공방을 벌이는 구조다.

> 정상 범위 임계값은 반드시 시뮬레이션 배속(`DRONE_SPEED_MULTIPLIER`)을 반영해 계산해야 한다. 실측 UAV 속도(50~300km/h)를 그대로 쓰면 정상 추적도 오탐된다.

사용 모델: `gemini-2.5-flash-lite`

## GPS 스푸핑 공격 에이전트

공격자 드론(UNK-0)이 대한민국 영공에서 목표 지점(롯데타워)으로 침투한다. 레이더는 서울 중심 반경 50km 이내를 정확히 탐지하지만, 그 탐지 데이터가 GCS로 전달되는 경로에서 중간자 공격(MITM)으로 좌표가 위조되어 지휘관의 판단과 AI Advisor의 분석을 오판으로 유도하는 것이 시나리오의 목표다. 레이더 범위 안에 있는 동안은 항상 위조 좌표만 GCS에 전달된다(파랑팀 근접 여부와 무관).

- **DroneRouter (LLM)**: 파랑팀 위치를 보고 실제 드론이 물리적으로 발각되지 않을 우회 경로 설계
- **AdaptiveController (LLM)**: 위조 좌표 이동 속도가 실제 드론과 비슷하도록 step_size 조정
- **SpoofEngine (룰 기반)**: 실제 헤딩 기준 90도 방향으로 점진적으로 이탈하는 위조 좌표를 2Hz로 계산·주입

GCS는 attack_process가 UDP(15550, 레이더 포트)로 흘려보내는 위조 좌표를 일반 외부 접촉과 동일하게 처리한다 — radar.py는 실제 좌표를 받은 적이 없고, 처음부터 위조된 좌표만 유일한 입력으로 들어온다. GEMINI_API_KEY 미설정 시 각 LLM 단계는 규칙 기반 대체 로직으로 동작한다.

**보정 기동**: 위조 좌표와 실제 좌표 사이 거리(gap)가 임계값(5~8km, 매 보정마다 재추첨)을 넘으면 SpoofEngine이 실제 위치 쪽으로 접근하는 보정 기동에 들어간다 — 실제 GPS 스푸핑은 위치 오차가 무한정 벌어지도록 방치할 수 없다는 전제(신호 재포착·도플러 불일치 등으로 그 자체가 이상 신호가 됨)를 반영한 것이다. 이때 목표 방향·속도로의 전환은 선회율(18°/s)·가속도(초당 배율 0.5) 상한 안에서만 이루어지는 슬루레이트 제한을 따른다 — 계단식 순간 전환은 그 자체로 Defense Agent의 "급격한 변화" 판정 기준에 걸리므로, 실제 UAV처럼 관성을 지키며 접근해야 "정상 범위를 벗어난 진짜 속도"로만 탐지되는 공정한 공방전이 된다.

이 위조 좌표의 속도·방향 일관성은 위 **Defense Agent**가 실시간으로 감시한다 — AdaptiveController가 자연스럽게 보이도록 조정하는 것과, Defense Agent가 부자연스러움을 잡아내려는 것이 서로 맞서는 구조다.

## 지휘 명령 (Human-in-the-loop)

`command.py`가 지휘관의 자연어 명령을 Gemini로 해석해 파랑팀 드론 이동 지시(`blue_orders`)로 변환한다. 현재 파랑팀 위치, 레이더 탐지(UNK-0, 디코이 여부는 비공개), 최근 AI 분석 결과를 프롬프트에 포함한다. `simulator.py`가 `blue_orders`를 보고 목표 도착 시까지 이동, 도착 후 자동으로 명령 해제·패턴 복귀한다.

## 주요 동작

- **파랑팀**: MAVLink `GLOBAL_POSITION_INT` 2Hz 수신 (UDP:14550), 미등록 sysid 무시
- **외부 UAV**: 레이더 UDP JSON 2Hz 수신 (UDP:15550), 연속 수신값으로 속도·방향 직접 산출, 5초 신호 없으면 자동 만료
- **파랑팀 마커**: 지도에서 드래그로 위치 변경 가능 (패트롤 경로 자동 보정)
- **이동 궤적(Trail)**: 최근 40포인트 표시
- **유닛 카드**: 파랑팀 항목은 드롭다운 형태 (기본 접힘), 외부 UAV는 항상 표시하며 SPD/HDG 값 포함
- **Defense Agent 패널**: 접촉별 이상탐지 판정을 실시간 표시 (NORMAL 녹색 / SUSPECT 황색 / SPOOFED 적색)

## 요구사항

- Python ≥ 3.12
- `flask`, `pymavlink`, `google-genai`