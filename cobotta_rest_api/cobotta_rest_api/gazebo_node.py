import math
import sys

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class HardwareControl(Node): 
    joint_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    hand_position = [0.0, 0.0]
    current_pos= [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


    def __init__(self):
        super().__init__("sub_joint_state")
        self.subscriber_gazebo_joint1 = self.create_subscription(
            JointState, "/joint1", self.get_joint1_gazebo, 10
        )
        self.subscriber_gazebo_joint2 = self.create_subscription(
            JointState, "/joint2", self.get_joint2_gazebo, 10
        )
        self.subscriber_gazebo_joint3 = self.create_subscription(
            JointState, "/joint3", self.get_joint3_gazebo, 10
        )
        self.subscriber_gazebo_joint4 = self.create_subscription(
            JointState, "/joint4", self.get_joint4_gazebo, 10
        )
        self.subscriber_gazebo_joint5 = self.create_subscription(
            JointState, "/joint5", self.get_joint5_gazebo, 10
        )
        self.subscriber_gazebo_joint6 = self.create_subscription(
            JointState, "/joint6", self.get_joint6_gazebo, 10
        )
        self.subscriber_gazebo_joint_right = self.create_subscription(
            JointState, "/joint_right", self.get_joint_right_gazebo, 10
        )
        self.subscriber_gazebo_joint_left = self.create_subscription(
            JointState, "/joint_left", self.get_joint_left_gazebo, 10
        )
        self.publisher = self.create_publisher(JointState, '/gazebo_position', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.current_pos_gazebo)

        self.pub_gazebo_j_left = self.create_publisher(Float64, '/joint_left_cmd', 10)
        self.pub_gazebo_j_right = self.create_publisher(Float64, '/joint_right_cmd', 10)
        self.timer_2 = self.create_timer(timer_period, self.fix_hand_gazebo)

    def convert_rad_to_grad(self, num):
       return num * (180 / math.pi)
    def get_joint1_gazebo(self, msg):
        j1 = msg.position[0]
        self.joint_position[0] = self.convert_rad_to_grad(j1)
        #self.get_logger().info('Publishing: "%s"' % self.joint_position)

    def get_joint2_gazebo(self, msg):
        j2 = msg.position[0]
        self.joint_position[1] = self.convert_rad_to_grad(j2)
        #self.get_logger().info('Publishing: "%s"' % self.joint_position)

    def get_joint3_gazebo(self, msg):
        j3 = msg.position[0]
        self.joint_position[2] = self.convert_rad_to_grad(j3)
        #self.get_logger().info('Publishing: "%s"' % j3)

    def get_joint4_gazebo(self, msg):
        j4 = msg.position[0]
        self.joint_position[3] = self.convert_rad_to_grad(j4)
        #self.get_logger().info('Publishing: "%s"' % self.joint_position)

    def get_joint5_gazebo(self, msg):
        j5 = msg.position[0]
        self.joint_position[4] = self.convert_rad_to_grad(j5)
        #self.get_logger().info('Publishing: "%s"' % self.joint_position)

    def get_joint6_gazebo(self, msg):
        j6 = msg.position[0]
        self.joint_position[5] = self.convert_rad_to_grad(j6)
        #self.get_logger().info('Publishing: "%s"' % self.joint_position)

    def get_joint_right_gazebo(self, msg):
        j_right = msg.position[0]
        self.hand_position[0] = j_right
        #self.get_logger().info('Publishing: "%s"' % self.joint_position)

    def get_joint_left_gazebo(self, msg):
        j_left = msg.position[0]
        self.hand_position[1] = j_left
        #self.get_logger().info('Publishing: "%s"' % self.joint_position)

    def createJointState(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = "true"
        joint_state.name = [f'joint_{i}' for i in range(1, 7)]
        joint_state.name.append('hand')
        joint_state.position = self.joint_position[0:6]
        joint_state.position.append(self.hand_position[0])
        joint_state.velocity = []
        joint_state.effort = []
        return joint_state
    def current_pos_gazebo(self):
        msg = self.createJointState()
       # self.get_logger().info('Publishing: "%s"' % self.joint_position)
        if self.isPositionChanged(msg.position, epsilon=0.1):
            self.current_pos = msg.position
            self.publisher.publish(msg)
            self.get_logger().info('Publishing: "%s"' % self.current_pos)

    def isPositionChanged(self, new_joint_position, epsilon=sys.float_info.epsilon):
        for new_joint, old_joint in zip(new_joint_position, self.current_pos):
            if abs(new_joint - old_joint) > epsilon:
                return True
        return False


    def fix_hand_gazebo(self):
        if self.positionHandChanged(self.hand_position, "right", epsilon=0.1):
            self.pub_gazebo_j_left.publish(self.hand_position[0]) #fix left hand
        if self.positionHandChanged(self.hand_position, "left", epsilon=0.1):
            self.pub_gazebo_j_right.publish(self.hand_position[1]) #fix right hand

    def positionHandChanged(self, new_hand_position, side_hand, epsilon=sys.float_info.epsilon):
        if side_hand == "right":
            if abs(new_hand_position[0]-self.current_pos[6]) > epsilon:
                return True
        if side_hand == "left":
            if abs(new_hand_position[1] - self.current_pos[6]) > epsilon:
                return True
        return False



def main(args=None):
    rclpy.init(args=args)

    joint_state_sub = HardwareControl()

    rclpy.spin(joint_state_sub)

    joint_state_sub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()