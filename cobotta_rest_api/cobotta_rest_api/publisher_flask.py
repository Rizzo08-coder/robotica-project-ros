import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from flask import Flask

app = Flask(__name__)

@app.route("/")
def cobottaPosition():
    rclpy.init()

    publisher_cobotta = PublisherCobotta()

    rclpy.spin(publisher_cobotta)

    publisher_cobotta.destroy_node()
    rclpy.shutdown()



class PublisherCobotta(Node):
    def __init__(self):
        super().__init__('publisher_cobotta')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        # timer_period = 1  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.timer_callback()

    def timer_callback(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4','joint_5','joint_6']
        joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        joint_state.velocity = []
        joint_state.effort = []
        self.publisher_.publish(joint_state)
        self.get_logger().info('Publishing: "%s"' % joint_state.name)
        self.i += 1


def main(args=None):
    app.run(debug=True, host="localhost")


if __name__ == '__main__':
    main()
