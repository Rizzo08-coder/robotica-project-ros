from sensor_msgs.msg import JointState
from flask import Blueprint, request

from ..publisher_flask import node
from ..publisher_flask import publisher


bp = Blueprint('api', __name__, url_prefix='/api')

@bp.route("/move-joints") #TODO: make in POST ?!
def moveCobotta():
    joint_delta = get_joints_delta_from_request()
    joint_state = JointState()
    joint_state.header.stamp = node.get_clock().now().to_msg()
    joint_state.header.frame_id = request.args.get('joint_abs', type=str)
    joint_state.name = [f'joint_{i}' for i in range(1, 7)]
    joint_state.position = joint_delta
    joint_state.velocity = []
    joint_state.effort = []
    publisher.publish(joint_state)
    node.get_logger().info('Publishing: "%s"' % joint_state.position)
    return { 'joint' : joint_delta ,
             'joint_abs' : joint_state.header.frame_id}

def get_joints_delta_from_request() :
    joint_delta = []
    for i in range(1, 7):
        joint_delta.append(request.args.get(f'joint_{i}', type=float))
    return joint_delta

