from sensor_msgs.msg import JointState
from flask import Blueprint, request


bp = Blueprint('api', __name__, url_prefix='/api')

@bp.route("/move-joints")
def moveCobotta():
    joint_1_delta = request.args.get('joint_1', type=float)
    joint_2_delta = request.args.get('joint_2', type=float)
    joint_3_delta = request.args.get('joint_3', type=float)
    joint_4_delta = request.args.get('joint_4', type=float)
    joint_5_delta = request.args.get('joint_5', type=float)
    joint_6_delta = request.args.get('joint_6', type=float)

    from .publisher_flask import node
    from .publisher_flask import publisher
    joint_state = JointState()
    joint_state.header.stamp = node.get_clock().now().to_msg()
    joint_state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
    joint_state.position = [joint_1_delta, joint_2_delta, joint_3_delta, joint_4_delta, joint_5_delta, joint_6_delta]
    joint_state.velocity = []
    joint_state.effort = []
    publisher.publish(joint_state)
    node.get_logger().info('Publishing: "%s"' % joint_state.name)
    return { 'joint_1': joint_1_delta,
             'joint_2': joint_2_delta,
             'joint_3': joint_3_delta,
             'joint_4': joint_4_delta,
             'joint_5': joint_5_delta,
             'joint_6': joint_6_delta
            }