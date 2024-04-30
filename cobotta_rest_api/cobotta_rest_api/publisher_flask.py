from threading import Thread

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from flask import Flask



rclpy.init(args=None)
node = rclpy.create_node("publisher_cobotta")

Thread(target=lambda:node).start()

publisher = node.create_publisher(JointState, '/joint_states', 10)

app = Flask(__name__)

@app.route("/")
def cobottaPosition():
    joint_state = JointState()
    joint_state.header.stamp = node.get_clock().now().to_msg()
    joint_state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
    joint_state.position = [10.0, 2.0, 3.0, 4.0, 10.0, 3.0]
    joint_state.velocity = []
    joint_state.effort = []
    publisher.publish(joint_state)
    node.get_logger().info('Publishing: "%s"' % joint_state.name)
    return "ci siamo riusciti"




def main(args=None):
    app.run(debug=True, host="localhost")



if __name__ == '__main__':
    main()
