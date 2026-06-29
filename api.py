import copy
from flask import Blueprint, jsonify, request
from config import PATHS
import state, ai, attack

bp = Blueprint('api', __name__)


@bp.route('/api/state')
def api_state():
    with state.lock:
        res = {'units': list(copy.deepcopy(state.units).values())}
    return jsonify(res)


@bp.route('/api/move', methods=['POST'])
def api_move():
    d   = request.get_json(force=True)
    uid = d.get('id')
    lat = d.get('lat')
    lon = d.get('lon')
    if uid and lat is not None and lon is not None:
        with state.lock:
            if uid in state.units:
                state.units[uid]['lat'] = lat
                state.units[uid]['lon'] = lon
                p = PATHS.get(uid, {})
                if p.get('type') == 'circle':
                    p['cx'], p['cy'] = lat, lon
                elif p.get('type') == 'patrol':
                    dlat = p['p2'][0] - p['p1'][0]
                    dlon = p['p2'][1] - p['p1'][1]
                    p['p1'] = [lat, lon]
                    p['p2'] = [lat + dlat, lon + dlon]
    return jsonify({'ok': True})


@bp.route('/api/ai/latest')
def api_ai_latest():
    return jsonify(ai.latest())


@bp.route('/api/attack/status')
def api_attack_status():
    return jsonify(attack.status())


@bp.route('/api/attack/target', methods=['POST'])
def api_attack_target():
    d  = request.get_json(force=True)
    tid = d.get('target_id', 'UNK-0')
    attack.set_target(tid)
    return jsonify({'ok': True, 'target_id': tid})