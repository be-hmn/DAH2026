#!/usr/bin/env python3
import logging
logging.getLogger('werkzeug').setLevel(logging.ERROR)

from flask import Flask, render_template
import simulator, receiver, radar, ai, attack, drone
from api import bp

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
    # 우리 드론 목표 좌표 — 서울 롯데타워
    TARGET = {'lat': 37.5125, 'lon': 127.1025}
    attack.start(target_id='UNK-0')          # spoofed_ids 등록 먼저
    drone.start(target=TARGET)               # 실제 드론 시뮬레이터 (unk_sim.py 대체)
    print('[GCS] http://127.0.0.1:8080')
    print('[GCS] MAVLink UDP:14550')
    print(f'[GCS] 드론 목표: 롯데타워 LAT {TARGET["lat"]} LON {TARGET["lon"]}')
    app.run(host='127.0.0.1', port=8080, threaded=True)