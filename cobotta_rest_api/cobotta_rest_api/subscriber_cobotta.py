import rclpy
from rclpy.node import Node

from .orin.bcapclient import BCAPClient as bcapclient

from sensor_msgs.msg import JointState
from my_robot_interfaces.srv import PositionJoint
from my_robot_interfaces.msg import PosJoint



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
        self.sub_joint_states  # prevent unused variable warnings
        self.pub_joint_states = self.create_publisher(PosJoint, '/actual_joint_position', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.current_position)
        self.current_joints_service = self.create_service(PositionJoint, '/get_position_joints', self.get_joints_callback)

    def current_position(self):
        msg = PosJoint()
        msg.position = self.m_bcapclient.robot_execute(self.HRobot, 'CurJnt')[0:6]
        msg.position.append(self.m_bcapclient.controller_execute(self.hCtrl, "HandCurPos"))
        self.pub_joint_states.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.position)

    def get_joints_callback(self, request, response):
        response.position = self.m_bcapclient.robot_execute(self.HRobot, 'CurJnt')[0:6]
        response.position.append(self.m_bcapclient.controller_execute(self.hCtrl, "HandCurPos"))
        return response

    def move_joint(self, j1=0, j2=0, j3=90, j4=0, j5=90, j6=0, hand=0, is_joints_abs="false"):
        self.current_joints_states = self.m_bcapclient.robot_execute(self.HRobot, 'CurJnt')[0:6]
        self.current_joints_states.append(self.m_bcapclient.controller_execute(self.hCtrl, "HandCurPos"))
        self.m_bcapclient.robot_execute(self.HRobot, "TakeArm")
        self.m_bcapclient.robot_execute(self.HRobot, "Motor", [1, 0])
        self.m_bcapclient.robot_execute(self.HRobot, "ExtSpeed", 80)
        if is_joints_abs == "false":
            self.m_bcapclient.robot_move(
                  self.HRobot, 1, "@P J({},{},{},{},{},{})".format(self.current_joints_states[0]+j1,
                                                                       self.current_joints_states[1]+j2,
                                                                       self.current_joints_states[2]+j3,
                                                                       self.current_joints_states[3]+j4,
                                                                       self.current_joints_states[4]+j5,
                                                                       self.current_joints_states[5]+j6 )
            )
            self.m_bcapclient.controller_execute(self.hCtrl, "HandMoveA", [self.current_joints_states[6] + hand,100])
        else:
            self.m_bcapclient.robot_move(
                self.HRobot, 1, "@P J({},{},{},{},{},{})".format(j1,j2,j3,j4,j5,j6)
            )
            self.m_bcapclient.controller_execute(self.hCtrl, "HandMoveA", [hand, 100])
        self.m_bcapclient.robot_execute(self.HRobot, "GiveArm")


    def my_timer_callback(self, joint_msg):
        is_joints_abs = joint_msg.header.frame_id
        hand = joint_msg.position[6]
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

        self.get_logger().info('Received')
        self.move_joint(j1, j2, j3, j4, j5, j6, hand, is_joints_abs)

def main(args=None):
    rclpy.init(args=args)

    joint_state_sub = HardwareControl()

    rclpy.spin(joint_state_sub)

    joint_state_sub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


