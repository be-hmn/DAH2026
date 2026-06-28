# DAH — GCS Tactical Display

MAVLink 기반 지상통제시스템(GCS) 전술 디스플레이. Flask 백엔드와 Leaflet.js 기반 웹 UI로 아군 유닛 위치를 실시간 추적하고 외부 UAV를 탐지한다.

## 아키텍처

```
[아군 UAV × 4]  ── MAVLink (UDP:14550) ──▶  receiver.py  ──┐
                                                             ├──▶ state.py ──▶ Flask API ──▶ 브라우저
[레이더 시스템]  ── UDP JSON (UDP:15550) ──▶  radar.py    ──┘
```

## 모듈 구성

| 파일 | 역할 |
|---|---|
| `gcs_system.py` | 진입점 — Flask 앱 + 스레드 시작 |
| `config.py` | 모든 상수 (유닛 정보, 경로, TTL, 포트) |
| `state.py` | 공유 상태 (`units`, `last_seen`, `Lock`) |
| `simulator.py` | 아군 MAVLink 패킷 송신 시뮬레이터 |
| `receiver.py` | MAVLink 수신 → 아군 위치 갱신 |
| `radar.py` | 레이더 피드 수신 → 외부 UAV 등록/만료 |
| `api.py` | Flask Blueprint (`/api/state`, `/api/move`) |
| `unk_sim.py` | 레이더 시스템 시뮬레이터 (UDP JSON) |
| `templates/index.html` | 전술 지도 UI (Leaflet, 위성영상) |

## 아군 유닛

| ID | 유형 | 경로 |
|---|---|---|
| ALPHA-1 | UAV | 원형 순환 |
| BRAVO-2 | UAV | 직선 왕복 |
| CHARLIE-3 | GND | 직선 왕복 (저속) |
| DELTA-4 | UAV | 원형 순환 |

## 설치 및 실행

```bash
# 의존성 설치
uv sync

# GCS 서버 실행
uv run gcs_system.py
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

`/api/move` 요청 형식:
```json
{ "id": "ALPHA-1", "lat": 37.534, "lon": 126.985 }
```

## 주요 동작

- **아군**: MAVLink `GLOBAL_POSITION_INT` 2Hz 수신 (UDP:14550), 미등록 sysid 수신 시 경고 후 무시
- **외부 UAV**: 레이더 UDP JSON 2Hz 수신 (UDP:15550), **5초** 신호 없으면 자동 만료 제거
- **아군 마커**: 지도에서 드래그로 위치 변경 가능 (패트롤 경로 자동 보정)
- **이동 궤적(Trail)**: 최근 40포인트 표시

## 요구사항

- Python ≥ 3.12
- `flask`, `pymavlink`