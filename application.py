from flask import Flask
from flask import request
from flask import send_from_directory
from api.trajectory import get_trajectory

application = Flask(__name__, static_url_path='', static_folder='web/build')

@application.route("/", defaults={'path':''})
def serve(path):
    return send_from_directory(application.static_folder,'index.html')

@application.route("/api/xt")
def api():
    target = request.args.get('target')
    target = list(map(float, target.split(',')))
    formation = request.args.get('formation')
    formation = list(map(float, formation.split(',')))
    r = float(request.args.get('r'))
    sep = float(request.args.get('sep'))
    x_t = get_trajectory(target, formation, r, sep)
    return {
        "trajectory": x_t
    }

if __name__ == '__main__':
    application.run(debug=False)