#!/usr/bin/env python3
import logging
logging.getLogger('werkzeug').setLevel(logging.ERROR)

from flask import Flask, render_template
import simulator, receiver, radar
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
    print('[GCS] http://127.0.0.1:8080')
    print('[GCS] MAVLink UDP:14550')
    app.run(host='127.0.0.1', port=8080, threaded=True)