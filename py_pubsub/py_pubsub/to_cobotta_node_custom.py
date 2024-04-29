import rclpy
from rclpy.node import Node

from orin.bcapclient import BCAPClient as bcapclient

from sensor_msgs.msg import JointState


class HardwareControl(Node):
    def __init__(self):
        # set IP Address , Port number and Timeout of connected RC8
        self.host = "192.168.0.1"
        self.port = 5007
        self.timeout = 2000

        # set Parameter
        self.Name = ""
        self.Provider = "CaoProv.DENSO.VRC"
        self.Machine = "localhost"
        self.Option = ""

        self.comp = 1
        self.loopflg = True
        self.ESC = 0x1B  # [ESC] virtual key code

        # Connection processing of tcp communication
        self.m_bcapclient = bcapclient(self.host, self.port, self.timeout)
        # start b_cap Service
        self.m_bcapclient.service_start("")

        # Connect to RC8 (RC8(VRC)provider)
        self.hCtrl = self.m_bcapclient.controller_connect(
            self.Name, self.Provider, self.Machine, self.Option
        )

        self.HRobot = self.m_bcapclient.controller_getrobot(self.hCtrl, "Arm", "")

        super().__init__("sub_joint_state")
        self.sub_joint_states = self.create_subscription(
            JointState, "/joint_states", self.my_timer_callback, 10
        )
        self.sub_joint_states  # prevent unused variable warning

    def move_joint(self, j1=0, j2=0, j3=90, j4=0, j5=90, j6=0):
        self.m_bcapclient.robot_execute(self.HRobot, "TakeArm")
        self.m_bcapclient.robot_execute(self.HRobot, "Motor", [1, 0])
        self.m_bcapclient.robot_execute(self.HRobot, "ExtSpeed", 100)
        self.m_bcapclient.robot_move(
            self.HRobot, 1, "@P J({},{},{},{},{},{})".format(j1, j2, j3, j4, j5, j6)
        )
        self.m_bcapclient.robot_execute(self.HRobot, "GiveArm")

    def my_timer_callback(self, joint_msg):
        for i in range(len(joint_msg.name)):
            if i == joint_msg.name.index("joint_1"):
                j1 = joint_msg.position[i]
            elif i == joint_msg.name.index("joint_2"):
                j2 = joint_msg.position[i]
            elif i == joint_msg.name.index("joint_3"):
                j3 = joint_msg.position[i]
            elif i == joint_msg.name.index("joint_4"):
                j4 = joint_msg.position[i]
            elif i == joint_msg.name.index("joint_5"):
                j5 = joint_msg.position[i]
            elif i == joint_msg.name.index("joint_6"):
                j6 = joint_msg.position[i]

        self.move_joint(j1, j2, j3, j4, j5, j6)

    def timer_callback(self, joint_msg):
        for i in range(len(joint_msg.name)):
            rviz_param_b = 0.0
            if i == joint_msg.name.index("joint_1"):
                rviz_param_a = -2.62 / 150
                j1 = (joint_msg.position[i] - rviz_param_b) / rviz_param_a
            elif i == joint_msg.name.index("joint_2"):
                rviz_param_a = 2.8 / 160
                j2 = (joint_msg.position[i] - rviz_param_b) / rviz_param_a
            elif i == joint_msg.name.index("joint_3"):
                rviz_param_a = -2.13 / 122
                rviz_param_b = 1.5713
                j3 = (joint_msg.position[i] - rviz_param_b) / rviz_param_a
            elif i == joint_msg.name.index("joint_4"):
                rviz_param_a = 2.97 / 170
                j4 = (joint_msg.position[i] - rviz_param_b) / rviz_param_a
            elif i == joint_msg.name.index("joint_5"):
                rviz_param_a = -4.02 / 230
                rviz_param_b = 1.5730
                j5 = (joint_msg.position[i] - rviz_param_b) / rviz_param_a
            elif i == joint_msg.name.index("joint_6"):
                rviz_param_a = 2.97 / 180
                j6 = (joint_msg.position[i] - rviz_param_b) / rviz_param_a

        self.move_joint(j1, j2, j3, j4, j5, j6)


def main(args=None):
    rclpy.init(args=args)

    joint_state_pub = HardwareControl()

    rclpy.spin(joint_state_pub)

    joint_state_pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

"""

Once, Full message:
ros2 topic pub --once /joint_states sensor_msgs/JointState "{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''}, name: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'], position: [0, 0, 90, 0, 90, 0], velocity: [], effort: []}"

Loop, Full message:
ros2 topic pub /joint_states sensor_msgs/JointState "{header: {seq: 1, stamp: {secs: 0, nsecs: 0}, frame_id: ''}, name: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'], position: [0, 0, 90, 0, 90, 0], velocity: [], effort: []}"

Loop, Only position:
ros2 topic pub /joint_states sensor_msgs/JointState "{name: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'], position: [0, 0, 90, 0, 90, 0]}"

Once, Only position:
ros2 topic pub --once /joint_states sensor_msgs/JointState "{name: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'], position: [0, 0, 90, 0, 90, 0]}"

"""
