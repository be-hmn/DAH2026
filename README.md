# DAH — GCS Tactical Display

MAVLink 기반 지상통제시스템(GCS) 전술 디스플레이. Flask 백엔드와 Leaflet.js 기반 웹 UI로 아군 유닛 위치를 실시간 추적하고, 레이더로 외부 UAV를 탐지하며, Gemini AI가 전장 변화를 자동 분석한다.

## 아키텍처

```
[아군 UAV × 4]  ── MAVLink (UDP:14550) ──▶  receiver.py  ──┐
                                                             ├──▶ state.py ──▶ Flask API ──▶ 브라우저
[레이더 시스템]  ── UDP JSON (UDP:15550) ──▶  radar.py    ──┘
                                                             │
                                             ai.py (상태 감시) ──▶ Gemini API
```

## 모듈 구성

| 파일 | 역할 |
|---|---|
| `gcs_system.py` | 진입점 — Flask 앱 + 전체 스레드 시작 |
| `config.py` | 모든 상수 (유닛 정보, 경로/속도, TTL, 포트) |
| `state.py` | 공유 상태 (`units`, `last_seen`, `Lock`) |
| `simulator.py` | 아군 MAVLink 패킷 송신 시뮬레이터 (2Hz) |
| `receiver.py` | MAVLink 수신 → 아군 위치 갱신 |
| `radar.py` | 레이더 피드 수신 → 외부 UAV 등록/만료 |
| `ai.py` | 상태 변화 감지 → Gemini 자동 분석 |
| `api.py` | Flask Blueprint (`/api/state`, `/api/move`, `/api/ai/latest`) |
| `unk_sim.py` | 레이더 시스템 시뮬레이터 (UDP JSON) |
| `templates/index.html` | 전술 지도 UI (Leaflet, 위성영상, KRDS 다크 테마) |

## 아군 유닛

| ID | 유형 | 경로 | 속도 (기준) |
|---|---|---|---|
| ALPHA-1 | UAV | 원형 순환 | ~235 km/h |
| BRAVO-2 | UAV | 직선 왕복 | ~226 km/h avg |
| CHARLIE-3 | GND | 직선 왕복 | ~43 km/h avg |
| DELTA-4 | UAV | 원형 순환 | ~224 km/h |

UAV 속도는 KUS-FS(한국군 전술 MALE UAV, 순항 250 km/h) 기준으로 산정. GND는 전술 UGV 야지 기준.

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
```

외부 UAV 시뮬레이션 (선택):

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

`/api/move` 요청 형식:
```json
{ "id": "ALPHA-1", "lat": 37.534, "lon": 126.985 }
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

## 주요 동작

- **아군**: MAVLink `GLOBAL_POSITION_INT` 2Hz 수신 (UDP:14550), 미등록 sysid 무시
- **외부 UAV**: 레이더 UDP JSON 2Hz 수신 (UDP:15550), 5초 신호 없으면 자동 만료
- **아군 마커**: 지도에서 드래그로 위치 변경 가능 (패트롤 경로 자동 보정)
- **이동 궤적(Trail)**: 최근 40포인트 표시
- **유닛 카드**: 아군 항목은 드롭다운 형태 (기본 접힘), 외부 UAV는 항상 표시

## 요구사항

- Python ≥ 3.12
- `flask`, `pymavlink`, `google-genai`