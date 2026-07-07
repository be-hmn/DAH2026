#!/usr/bin/env python3
import logging
logging.getLogger('werkzeug').setLevel(logging.ERROR)

from flask import Flask, render_template
import simulator, receiver, radar, ai, attack_link, drone, defense_agent
from api import bp
from config import DEFENSE_AGENT_ENABLED

app = Flask(__name__)
app.register_blueprint(bp)

@app.route('/')
def index():
    return render_template('index.html')


if __name__ == '__main__':
    receiver.start()
    simulator.start()
    radar.start()
    ai.start()
    if DEFENSE_AGENT_ENABLED:
        defense_agent.start()
        print('[GCS] Defense Agent 활성화 — 이상탐지 판단 수행')
    else:
        print('[GCS] Defense Agent 비활성화 (DEFENSE_AGENT_ENABLED=0) — 공격 에이전트만 단독 동작, 비교 실험용')
    # 우리 드론 목표 좌표 — 서울 롯데타워 (attack_process.py 에도 동일하게 설정되어 있어야 함)
    TARGET = {'lat': 37.5125, 'lon': 127.1025}
    drone.start(target=TARGET)               # 실제 드론 시뮬레이터 (unk_sim.py 대체)
    attack_link.start()                      # attack_process.py(별도 프로세스)와 UDP 연결
    print('[GCS] http://127.0.0.1:8080')
    print('[GCS] MAVLink UDP:14550')
    print(f'[GCS] 드론 목표: 롯데타워 LAT {TARGET["lat"]} LON {TARGET["lon"]}')
    print('[GCS] attack_process.py 를 별도로 실행해야 GPS 스푸핑이 동작합니다')
    app.run(host='127.0.0.1', port=8080, threaded=True)