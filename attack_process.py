#!/usr/bin/env python3
"""
attack_process.py — GPS 스푸핑 공격 에이전트, GCS와 별개의 독립 프로세스로 구동.

MITM 구조:
  GCS(drone.py 실제 위치 + 파랑팀 위치) ──UDP 텔레메트리(ATTACK_IN_PORT)──▶ 여기
  여기(LLM으로 위조 좌표 계산) ──UDP(RADAR_PORT)──▶ GCS radar.py (일반 접촉으로 처리, 원래 unk_sim.py 자리)
  여기(DroneRouter 우회 경로) ──UDP(ATTACK_OUT_PORT)──▶ GCS(drone.py 웨이포인트 갱신)
  여기(status) ──UDP(ATTACK_OUT_PORT)──▶ GCS(UI 상태 캐시)

실행: uv run --env-file .env attack_process.py
"""
import json, os, socket, threading, time

import attack
from config import ATTACK_IN_PORT, ATTACK_OUT_PORT, RADAR_PORT

GCS_HOST = '127.0.0.1'
TARGET   = {'lat': 37.5125, 'lon': 127.1025}   # 실제 드론 최종 목표 — 서울 롯데타워

_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

_lock        = threading.Lock()
_blues: list = []
_real_pos: dict = {}
_wp_remaining: int = 0


def _get_telemetry():
    with _lock:
        if not _real_pos:
            return None
        return _real_pos['lat'], _real_pos['lon'], list(_blues), _wp_remaining


def _send_decoy(uid: str, lat: float, lon: float):
    pkt = {'id': uid, 'lat': round(lat, 7), 'lon': round(lon, 7)}
    try:
        _sock.sendto(json.dumps(pkt).encode(), (GCS_HOST, RADAR_PORT))
    except Exception as e:
        print(f'[ATTACK-PROC] decoy 송신 오류: {e}')


def _send_waypoints(points: list[tuple[float, float]]):
    pkt = {'type': 'waypoints', 'points': [[p[0], p[1]] for p in points]}
    try:
        _sock.sendto(json.dumps(pkt).encode(), (GCS_HOST, ATTACK_OUT_PORT))
    except Exception as e:
        print(f'[ATTACK-PROC] waypoints 송신 오류: {e}')


def _send_status(status: dict):
    pkt = {'type': 'status', **status}
    try:
        _sock.sendto(json.dumps(pkt).encode(), (GCS_HOST, ATTACK_OUT_PORT))
    except Exception as e:
        print(f'[ATTACK-PROC] status 송신 오류: {e}')


def _receive(sock):
    global _blues, _real_pos, _wp_remaining
    print(f'[ATTACK-PROC] 텔레메트리 수신 대기 (UDP:{ATTACK_IN_PORT})')
    while True:
        try:
            data, _ = sock.recvfrom(65536)
            obj  = json.loads(data.decode())
            kind = obj.get('type')
            if kind == 'telemetry':
                with _lock:
                    _blues        = obj.get('blues', [])
                    _real_pos     = obj.get('drone', {})
                    _wp_remaining = obj.get('wp_remaining', 0)
                if obj.get('mission_complete'):
                    print('[ATTACK-PROC] 임무 완료 신호 수신 — 스푸핑 종료')
                    attack.stop()
                    os._exit(0)
            elif kind == 'control':
                cmd = obj.get('cmd')
                if cmd == 'set_target':
                    tid = obj.get('target_id', 'UNK-0')
                    print(f'[ATTACK-PROC] 컨트롤: target 변경 → {tid}')
                    attack.set_target(tid)
        except socket.timeout:
            continue
        except Exception as e:
            print(f'[ATTACK-PROC] 수신 오류: {e}')


if __name__ == '__main__':
    in_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        in_sock.bind(('0.0.0.0', ATTACK_IN_PORT))
    except OSError as e:
        raise SystemExit(
            f'[ATTACK-PROC] UDP:{ATTACK_IN_PORT} 바인드 실패 ({e}) — '
            f'이미 실행 중인 attack_process.py 가 있는지 확인하세요 (예: pkill -f attack_process.py)'
        )
    in_sock.settimeout(1.0)

    threading.Thread(target=_receive, args=(in_sock,), daemon=True).start()
    attack.configure(
        get_telemetry=_get_telemetry,
        send_decoy=_send_decoy,
        send_waypoints=_send_waypoints,
        send_status=_send_status,
        target=TARGET,
    )
    attack.start('UNK-0')
    print(f'[ATTACK-PROC] 시작 — 목표: 롯데타워 LAT {TARGET["lat"]} LON {TARGET["lon"]}')
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        attack.stop()
        print('[ATTACK-PROC] 종료')