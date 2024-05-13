import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from my_robot_interfaces.msg import PosJoint

class HardwareControl(Node):
    joint_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    def __init__(self):
        super().__init__("sub_joint_state")
        self.subscriber_gazebo_joint1 =  self.create_subscription(
            JointState, "/joint1", self.get_joint1_gazebo, 10
        )
        self.subscriber_gazebo_joint1 = self.create_subscription(
            JointState, "/joint2", self.get_joint2_gazebo, 10
        )
        self.subscriber_gazebo_joint1 = self.create_subscription(
            JointState, "/joint3", self.get_joint3_gazebo, 10
        )
        self.subscriber_gazebo_joint1 = self.create_subscription(
            JointState, "/joint4", self.get_joint4_gazebo, 10
        )
        self.subscriber_gazebo_joint1 = self.create_subscription(
            JointState, "/joint5", self.get_joint5_gazebo, 10
        )
        self.subscriber_gazebo_joint1 = self.create_subscription(
            JointState, "/joint6", self.get_joint6_gazebo, 10
        )
        self.subscriber_gazebo_joint1 = self.create_subscription(
            JointState, "/joint_right", self.get_joint_right_gazebo, 10
        )
        self.subscriber_gazebo_joint1 = self.create_subscription(
            JointState, "/joint_left", self.get_joint_left_gazebo, 10
        )
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.current_pos_gazebo)

    def get_joint1_gazebo(self, msg):
        j1 = msg.position[0]
        self.joint_position[0] = j1
        #self.get_logger().info('Publishing: "%s"' % self.joint_position)

    def get_joint2_gazebo(self, msg):
        j2 = msg.position[0]
        self.joint_position[1] = j2
        #self.get_logger().info('Publishing: "%s"' % self.joint_position)

    def get_joint3_gazebo(self, msg):
        j3 = msg.position[0]
        self.joint_position[2] = j3
        #self.get_logger().info('Publishing: "%s"' % self.joint_position)

    def get_joint4_gazebo(self, msg):
        j4 = msg.position[0]
        self.joint_position[3] = j4
        #self.get_logger().info('Publishing: "%s"' % self.joint_position)

    def get_joint5_gazebo(self, msg):
        j5 = msg.position[0]
        self.joint_position[4] = j5
        #self.get_logger().info('Publishing: "%s"' % self.joint_position)

    def get_joint6_gazebo(self, msg):
        j6 = msg.position[0]
        self.joint_position[5] = j6
        #self.get_logger().info('Publishing: "%s"' % self.joint_position)

    def get_joint_right_gazebo(self, msg):
        j_right = msg.position[0]
        self.joint_position[6] = j_right
        #self.get_logger().info('Publishing: "%s"' % self.joint_position)

    def get_joint_left_gazebo(self, msg):
        j_left = msg.position[0]
        self.joint_position[7] = j_left
        #self.get_logger().info('Publishing: "%s"' % self.joint_position)

    def createJointState(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [f'joint_{i}' for i in range(1, 7)]
        joint_state.position = self.joint_position[0:6]
        joint_state.velocity = []
        joint_state.effort = []
        return joint_state
    def current_pos_gazebo(self):
       msg = self.createJointState()
       self.publisher.publish(msg)
       self.get_logger().info('Publishing: "%s"' % msg.position)





def main(args=None):
    rclpy.init(args=args)

    joint_state_sub = HardwareControl()

    rclpy.spin(joint_state_sub)

    joint_state_sub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()