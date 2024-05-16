import rclpy
from rclpy.node import Node

from my_robot_interfaces.srv import ListPosJoint

class TrajectoryExeNode(Node):
    def __init__(self):
        super().__init__("trajectory_exe_node")
        self.play_trajectory_service = self.create_service(ListPosJoint, '/play_trajectory',
                                                          self.play_trajectory_callback)

    def play_trajectory_callback(self, request, response):
        for joint_state in request.joints_position:
            self.get_logger().info('received')
            #TODO: move cobotta, not implemented yet
        response.completed = True
        return response

def main(args=None):
    rclpy.init(args=args)

    joint_state_sub = TrajectoryExeNode()

    rclpy.spin(joint_state_sub)

    joint_state_sub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


