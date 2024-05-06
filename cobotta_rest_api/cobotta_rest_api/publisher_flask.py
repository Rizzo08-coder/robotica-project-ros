from threading import Thread

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from my_robot_interfaces.srv import PositionJoint
from my_robot_interfaces.msg import PosJoint

from flask import Flask
from flask_socketio import SocketIO
from flask_cors import CORS


class FlaskNode(Node):
    def __init__(self):
        rclpy.init()
        super().__init__("sub_joint_state")
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.subscriber = self.create_subscription(
            PosJoint, "/actual_joint_position", self.actual_position_callback, 10
        )
        self.client = self.create_client(PositionJoint, '/get_position_joints')

    def actual_position_callback(self, msg): #emit msg on web socket (event: robot_position)
        #actual_position = msg.position
        actual_position = [1,1,1,1,1,1]
        socketio.emit('robot_position', actual_position)


flask_pub = FlaskNode()

def sendRequestPosition():
    '''
    req = PositionJoint.Request()
    future = flask_pub.client.call(req)
    return future.position
    '''
    return [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]





app = Flask(__name__)
socketio = SocketIO(app)
CORS(app, resources={r"/api/*": {"origins": "*"}})

from . import db
db.init_app(app)

from .blueprints import flask_api
app.register_blueprint(flask_api.bp)


def main(args=None):
    Thread(target=lambda: rclpy.spin(flask_pub)).start()
    socketio.run(app, debug=True, host="localhost")
    flask_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
