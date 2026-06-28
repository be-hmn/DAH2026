# DAH — GCS Tactical Display

MAVLink 기반 지상통제시스템(GCS) 전술 디스플레이. Flask 백엔드와 Leaflet.js 기반 웹 UI로 아군 유닛 위치를 실시간 추적하고 미식별 비행체를 탐지한다.

## 아키텍처

```
unk_sim.py ──┐
             │  UDP:14550
gcs_system.py (MAVLink 수신) → Flask API → 브라우저 (Leaflet 지도)
             │
MAVLink 시뮬레이터 (내장, 아군 4기)
```

| 컴포넌트 | 역할 |
|---|---|
| `gcs_system.py` | Flask 서버 + MAVLink 수신/송신 + 미식별 추적 |
| `unk_sim.py` | 미식별 비행체 시뮬레이터 |
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
# 의존성 설치 (uv 권장)
uv sync

# GCS 서버 실행
uv run gcs_system.py
# → http://127.0.0.1:8080
# → MAVLink UDP:14550 수신 대기
```

미식별 비행체 시뮬레이션 (선택):

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

- **MAVLink GLOBAL_POSITION_INT** 2Hz 수신 (UDP:14550)
- 미식별 비행체(UNK): sysid 매핑 불일치 시 자동 등록, **5초** 신호 없으면 만료 제거
- 아군 마커: 지도에서 드래그로 위치 변경 가능 (패트롤 경로 자동 보정)
- 이동 궤적(Trail): 최근 40포인트 표시

## 요구사항

- Python ≥ 3.12
- `flask`, `pymavlink`, `playwright`