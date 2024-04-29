import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from flask import Flask

app = Flask(__name__)

@app.route("/list")
def sendList():
    return {"list" : ["1","2","3"] }

class PublisherCobotta(Node):
    def __init__(self):
        super().__init__('publisher_cobotta')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4','joint5','joint6']
        joint_state.position = [3.0, 0.5, 1.5, 4.0, 2.0, 1.0]
        joint_state.velocity = []
        joint_state.effort = []
        self.publisher_.publish(joint_state)
        self.get_logger().info('Publishing: "%s"' % joint_state.name)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    publisher_cobotta = PublisherCobotta()

    rclpy.spin(publisher_cobotta)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher_cobotta.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    app.run(debug=True, host="localhost")
    main()
