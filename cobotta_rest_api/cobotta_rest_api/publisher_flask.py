from threading import Thread

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from my_robot_interfaces.srv import PositionJoint

from flask import Flask


class FlaskNode(Node):
    def __init__(self):
        rclpy.init()
        super().__init__("sub_joint_state")
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.cli = self.create_client(PositionJoint, '/get-position-joints')


flask_pub = FlaskNode()


'''
def sendRequestPosition():
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service non disponibile, riprovo...')
'''


app = Flask(__name__)

from .blueprints import flask_api
app.register_blueprint(flask_api.bp)


def main(args=None):
    Thread(target=lambda: rclpy.spin(flask_pub)).start()
    app.run(debug=True, host="localhost")
    flask_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
