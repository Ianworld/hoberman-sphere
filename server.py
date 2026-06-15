"""Interactive Hoberman sphere designer -- browser version.

    ./.venv/bin/python server.py        then open http://127.0.0.1:8765

Prefer app.py for a native desktop window with no server.
"""

import os

from flask import Flask, jsonify, request, Response

import designer as D

HERE = os.path.dirname(os.path.abspath(__file__))
app = Flask(__name__)


@app.route('/')
def index():
    return Response(D.load_ui(), mimetype='text/html')


@app.route('/build', methods=['POST'])
def build_route():
    r = D.do_build(request.json)
    r.pop('stl_print', None)       # served separately via /mesh
    r.pop('stl_open', None)
    return jsonify(**r)


@app.route('/mesh')
def mesh_route():
    state = request.args.get('state')
    gen = int(request.args.get('gen', -1))
    if D.CACHE.get('gen') != gen or state not in D.CACHE.get('stl', {}):
        return Response(status=409)
    return Response(D.CACHE['stl'][state],
                    mimetype='application/octet-stream')


@app.route('/check', methods=['POST'])
def check_route():
    return jsonify(**D.start_check())


@app.route('/check_status')
def check_status():
    return jsonify(**D.check_status())


@app.route('/export', methods=['POST'])
def export_route():
    return jsonify(**D.start_export())


@app.route('/export_status')
def export_status():
    return jsonify(**D.export_status())


if __name__ == '__main__':
    app.run(host='127.0.0.1', port=int(os.environ.get('PORT', 8765)),
            threaded=True)
