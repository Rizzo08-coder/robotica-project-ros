import rclpy
from rclpy.node import Node

from my_robot_interfaces.msg import PosJoint

class Publisher(Node):
    def __init__(self):
        super().__init__("sub_joint_state")
        self.pub_joint_states = self.create_publisher(PosJoint, '/actual_joint_position', 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.current_position)

    def current_position(self):
        msg = PosJoint()
        msg.position = [2.0,2.0,2.0,2.0,2.0,2.0]
        self.pub_joint_states.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.position)


def main(args=None):
    rclpy.init(args=args)

    joint_state_sub = Publisher()

    rclpy.spin(joint_state_sub)

    joint_state_sub.destroy_node()
    rclpy.shutdown()
