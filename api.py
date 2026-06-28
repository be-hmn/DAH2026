import copy
from flask import Blueprint, jsonify, request
from config import PATHS
import state

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