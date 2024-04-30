from threading import Thread

import rclpy
from sensor_msgs.msg import JointState
from flask import Flask

def create_publisher_node():
    rclpy.init(args=None)
    node = rclpy.create_node("publisher_cobotta_flask")
    Thread(target=lambda: node).start()
    return node

node = create_publisher_node()
publisher = node.create_publisher(JointState, '/joint_states', 10)

app = Flask(__name__)

from .blueprints import flask_api

app.register_blueprint(flask_api.bp)


def main(args=None):
    app.run(debug=True, host="localhost")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
