from sensor_msgs.msg import JointState
from flask import Blueprint, request

from ..db import get_db
from ..publisher_flask import flask_pub
from ..publisher_flask import sendRequestPosition


bp = Blueprint('api', __name__, url_prefix='/api')

@bp.route("/move-joints") #TODO: make in POST ?!
def moveCobotta():
    joint_delta = get_joints_delta_from_request()
    joint_state = JointState()
    joint_state.header.stamp = flask_pub.get_clock().now().to_msg()
    joint_state.header.frame_id = request.args.get('joint_abs', type=str)
    joint_state.name = [f'joint_{i}' for i in range(1, 7)]
    joint_state.position = joint_delta
    joint_state.velocity = []
    joint_state.effort = []
    flask_pub.publisher.publish(joint_state)
    flask_pub.get_logger().info('Publishing: "%s"' % joint_state.position)
    print(sendRequestPosition())
    return { 'joint' : joint_delta ,
             'joint_abs' : joint_state.header.frame_id}

def get_joints_delta_from_request() :
    joint_delta = []
    for i in range(1, 7):
        joint_delta.append(request.args.get(f'joint_{i}', type=float))
    return joint_delta


@bp.route("/create-trajectory")
def createTrajectory():
    name = request.args.get('name', type=str)
    db = get_db()
    db.execute("INSERT INTO trajectories (name) values (?)", (name,))
    db.commit()
    trajectory = db.execute("SELECT * FROM trajectories WHERE name=?", (name,)).fetchone()
    return {'id': trajectory['id'],
            'name': trajectory['name']}

@bp.route("/trajectories")
def getTrajectories():
    db = get_db()
    trajectories = db.execute("SELECT * FROM trajectories").fetchall()
    return {'result':[{'id': trajectory['id'],
            'name': trajectory['name']} for trajectory in trajectories]}

@bp.route("/trajectory/:id/save-point") #parametri: traiettoria (id), Joints

@bp.route("/points/:id") #parametri: punto (id) -> metodo DELETE

@bp.route("/trajectory/:id") #parametri: traiettoria (id)

@bp.route("/trajectory/:id/play") #parametri: traiettoria (id)

@bp.route("/joints-position") #chiama service




