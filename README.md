# DAH — GCS Tactical Display

MAVLink 기반 지상통제시스템(GCS) 전술 디스플레이. Flask 백엔드와 Leaflet.js 기반 웹 UI로 아군(파랑팀) 유닛 위치를 실시간 추적하고, 레이더로 외부 UAV를 탐지하며, Gemini AI가 전장 변화를 자동 분석·지휘관 명령을 해석한다. 별도 프로세스로 구동되는 GPS 스푸핑 공격 에이전트가 레이더→GCS 전달 경로를 중간자 공격(MITM)으로 조작해 지휘관과 AI Advisor의 오판을 유도하는 시나리오를 포함한다.

## 아키텍처

```
[파랑팀 UAV × 4]  ── MAVLink (UDP:14550) ──▶  receiver.py  ──┐
                                                               │
[GCS 프로세스: gcs_system.py]                                 ├──▶ state.py ──▶ Flask API ──▶ 브라우저
  simulator.py  (파랑팀 MAVLink 송신, blue_orders 추종)       │
  drone.py      (침투 드론 UNK-0 실제 위치 시뮬레이션)         │
  radar.py      (UDP:15550 수신 → 외부 UAV 등록/만료) ─────────┘
  ai.py         (상태 감시 → Gemini 자동 분석)
  command.py    (지휘관 자연어 명령 → Gemini → blue_orders)
  attack_link.py(attack_process 와 UDP 연결)
        │  ▲
        │  │ UDP 텔레메트리(15551): 파랑팀 위치 + UNK-0 실제 위치
        ▼  │ UDP(15552): status / 우회 waypoints
[attack_process.py — 독립 프로세스]
  attack/agent.py — GPS 스푸핑 공격 에이전트 (LLM 2종 + 룰 기반 스푸핑 엔진)
        │
        ▼ UDP:15550 (위조 좌표 주입 — radar.py 는 일반 외부 접촉으로 처리, MITM 지점)
   GCS radar.py
```

GCS와 attack_process는 메모리를 공유하지 않고 UDP JSON으로만 통신한다 — attack_process가 죽어도 GCS는 정상 동작(스푸핑만 비활성)하며, GCS 재시작 중에도 attack_process는 독립적으로 유지된다.

## 모듈 구성

| 파일 | 역할 |
|---|---|
| `gcs_system.py` | 진입점 — Flask 앱 + GCS 쪽 전체 스레드 시작 |
| `config.py` | 모든 상수 (유닛 정보, 경로/속도, TTL, 포트) |
| `state.py` | GCS 공유 상태 (`units`, `blue_orders`, `attack_status`, `Lock` 등) |
| `simulator.py` | 파랑팀 MAVLink 송신 시뮬레이터 (2Hz), `blue_orders` 있으면 지휘 명령 우선 추종 |
| `receiver.py` | MAVLink 수신 → 파랑팀 위치 갱신 |
| `radar.py` | 레이더 포트(UDP:15550) 수신 → 외부 UAV 등록/만료 |
| `drone.py` | 침투 드론(UNK-0) 실제 위치 시뮬레이션 — DMZ→목표(롯데타워), 우회 웨이포인트 추종 |
| `ai.py` | 상태 변화 감지 → Gemini 전술 분석 (판단/권고) |
| `command.py` | 파랑팀 지휘관 자연어 명령 → Gemini 해석 → `blue_orders` |
| `attack_link.py` | GCS ↔ attack_process 간 UDP 연결 (텔레메트리 송신, status/waypoints 수신) |
| `api.py` | Flask Blueprint (`/api/state`, `/api/move`, `/api/ai/*`, `/api/attack/*`, `/api/command/*`) |
| `attack/agent.py` | GPS 스푸핑 공격 에이전트 로직 (DroneRouter LLM + 스푸핑 엔진) |
| `attack_process.py` | 공격 에이전트 독립 실행 프로세스 (GCS와 UDP로만 통신) |
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

# GCS 서버 실행
uv run --env-file .env gcs_system.py
# → http://127.0.0.1:8080
# → MAVLink UDP:14550 수신 대기
# → 레이더 UDP:15550 수신 대기

# GPS 스푸핑 공격 에이전트 — 별도 프로세스 (GCS와 UDP로만 통신, 미실행 시 스푸핑 비활성)
uv run --env-file .env attack_process.py
```

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

## GPS 스푸핑 공격 에이전트

공격자 드론(UNK-0)이 대한민국 영공에서 목표 지점(롯데타워)으로 침투한다. 레이더는 서울 중심 반경 50km 이내를 정확히 탐지하지만, 그 탐지 데이터가 GCS로 전달되는 경로에서 중간자 공격(MITM)으로 좌표가 위조되어 지휘관의 판단과 AI Advisor의 분석을 오판으로 유도하는 것이 시나리오의 목표다. 레이더 범위 안에 있는 동안은 항상 위조 좌표만 GCS에 전달된다(파랑팀 근접 여부와 무관).

- **DroneRouter (LLM)**: 파랑팀 위치를 보고 실제 드론이 물리적으로 발각되지 않을 우회 경로 설계
- **AdaptiveController (LLM)**: 위조 좌표 이동 속도가 실제 드론과 비슷하도록 step_size 조정
- **SpoofEngine (룰 기반)**: 실제 헤딩 기준 90도 방향으로 점진적으로 이탈하는 위조 좌표를 2Hz로 계산·주입

GCS는 attack_process가 UDP(15550, 레이더 포트)로 흘려보내는 위조 좌표를 일반 외부 접촉과 동일하게 처리한다 — radar.py는 실제 좌표를 받은 적이 없고, 처음부터 위조된 좌표만 유일한 입력으로 들어온다. GEMINI_API_KEY 미설정 시 각 LLM 단계는 규칙 기반 대체 로직으로 동작한다.

## 지휘 명령 (Human-in-the-loop)

`command.py`가 지휘관의 자연어 명령을 Gemini로 해석해 파랑팀 드론 이동 지시(`blue_orders`)로 변환한다. 현재 파랑팀 위치, 레이더 탐지(UNK-0, 디코이 여부는 비공개), 최근 AI 분석 결과를 프롬프트에 포함한다. `simulator.py`가 `blue_orders`를 보고 목표 도착 시까지 이동, 도착 후 자동으로 명령 해제·패턴 복귀한다.

## 주요 동작

- **파랑팀**: MAVLink `GLOBAL_POSITION_INT` 2Hz 수신 (UDP:14550), 미등록 sysid 무시
- **외부 UAV**: 레이더 UDP JSON 2Hz 수신 (UDP:15550), 5초 신호 없으면 자동 만료
- **파랑팀 마커**: 지도에서 드래그로 위치 변경 가능 (패트롤 경로 자동 보정)
- **이동 궤적(Trail)**: 최근 40포인트 표시
- **유닛 카드**: 파랑팀 항목은 드롭다운 형태 (기본 접힘), 외부 UAV는 항상 표시

## 요구사항

- Python ≥ 3.12
- `flask`, `pymavlink`, `google-genai`