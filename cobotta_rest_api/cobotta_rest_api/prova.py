import rclpy
from rclpy.node import Node


from sensor_msgs.msg import JointState

from my_robot_interfaces.msg import PosJoint

class HardwareControl(Node):
    def __init__(self):
        super().__init__("sub_joint_state")
        self.pub_joint_states = self.create_publisher(PosJoint, '/actual_joint_position', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.current_position)


    def current_position(self):
        msg = PosJoint()
        msg.position = [100.0, 20.0, 30.0, 40.0, 10.0, 15.0, 5.3]
        self.pub_joint_states.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.position)


def main(args=None):
    rclpy.init(args=args)

    joint_state_sub = HardwareControl()

    rclpy.spin(joint_state_sub)

    joint_state_sub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()