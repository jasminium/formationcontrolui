from flask import Flask, request, send_from_directory
from trajectory import get_trajectory

app = Flask(__name__)

"""
@app.route('/<path:path>')
def send_js(path):
    return send_from_directory('static', path)

@app.route('/')
def root():
    return app.send_static_file('index.html')
"""

@app.route('/api/xt')
def api():
    
    target = request.args.get('target')
    target = list(map(float, target.split(',')))
    print(target)
    formation = request.args.get('formation')
    formation = list(map(float, formation.split(',')))
    r = float(request.args.get('r'))
    sep = float(request.args.get('sep'))
    print(target, formation, r, sep)
    x_t = get_trajectory(target, formation, r, sep)
    return {
        "trajectory": x_t
    }