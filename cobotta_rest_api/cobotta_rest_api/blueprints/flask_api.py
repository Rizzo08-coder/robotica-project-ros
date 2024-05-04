from sensor_msgs.msg import JointState
from flask import Blueprint, request

from ..db import get_db
from ..publisher_flask import flask_pub
from ..publisher_flask import sendRequestPosition


bp = Blueprint('api', __name__, url_prefix='/api')

@bp.route("/move-joints") #TODO: make in POST ?!
def moveCobotta():
    joint_delta = get_joints_delta_from_request()
    joint_state = createJointState(joint_delta)
    flask_pub.publisher.publish(joint_state)
    flask_pub.get_logger().info('Publishing: "%s"' % joint_state.position)
    actual_joints_position = sendRequestPosition()
    return { 'position' : actual_joints_position }

def get_joints_delta_from_request() :
    joint_delta = []
    for i in range(1, 7):
        joint_delta.append(request.args.get(f'joint_{i}', type=float))
    return joint_delta

def createJointState(joint_delta):
    joint_state = JointState()
    joint_state.header.stamp = flask_pub.get_clock().now().to_msg()
    joint_state.header.frame_id = request.args.get('joint_abs', type=str)
    joint_state.name = [f'joint_{i}' for i in range(1, 7)]
    joint_state.position = joint_delta
    joint_state.velocity = []
    joint_state.effort = []
    return joint_state



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

@bp.route("/trajectory/<int:id>/save-point") #parametri: traiettoria (id), Joints
def savePoint(id):
    trajectory_id = id
    robot_position = sendRequestPosition()
    db = get_db()
    db.execute("INSERT INTO points (j1,j2,j3,j4,j5,j6,trajectory_id) values (?,?,?,?,?,?,?)",
               (robot_position[0], robot_position[1], robot_position[2], robot_position[3],
                robot_position[4], robot_position[5], trajectory_id))
    db.commit()
    return { 'point' : robot_position }



@bp.route("/points")
def getPoints(): #debug method
    db=get_db()
    points = db.execute("SELECT * FROM points").fetchall()
    return {'result': [{'id': point['id'],
                        'j1': point['j1'],
                         't_id': point['trajectory_id']} for point in points]}


@bp.route("/points/<int:id>") #parametri: punto (id) -> metodo DELETE
def deletePoint():
    return {}

@bp.route("/trajectory/:id") #parametri: traiettoria (id)
def showTrajectory():
    return {}

@bp.route("/trajectory/:id/play") #parametri: traiettoria (id)
def playTrajectory():
    return {}

@bp.route("/joints-position") #chiama service
def getJointsPosition():
    return {}



