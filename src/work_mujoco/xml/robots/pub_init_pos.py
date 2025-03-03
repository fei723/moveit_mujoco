import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

open_pos = [
    -1.492257,
    -0.3,
    0.6,
    0.6,
    0.0,
    0.0,
]

close_pos = [
    -1.492257,
    -0.4,
    0.75,
    0.75,
    0.0,
    0.0,
]

class JointTrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('joint_trajectory_publisher')

        # 创建一个发布者，发布到 joint_trajectory_controller 的 command 话题
        self.publisher_ = self.create_publisher(JointTrajectory, '/right_hand_controller/joint_trajectory', 10)

        # 设置定时器，每隔1秒发送一次消息
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # 创建 JointTrajectory 消息
        msg = JointTrajectory()
        msg.joint_names = [
            "r_thumb_cmc_y_joint",
            "r_thumb_cmc_p_joint",
            "r_index_mcp_joint",
            "r_middle_mcp_joint",
            "r_ring_mcp_joint",
            "r_pinky_mcp_joint",
        ]  # 替换成你机器人对应的关节名称

        # 创建 JointTrajectoryPoint 消息，设置目标位置为 0
        point = JointTrajectoryPoint()
        point.positions = open_pos
        point.time_from_start.sec = 1  # 设置此点的持续时间（单位：秒）

        # 将该点添加到 JointTrajectory 消息中
        msg.points = [point]

        # 发布消息
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing JointTrajectory with 0 positions')

def main(args=None):
    rclpy.init(args=args)

    # 创建节点并运行
    joint_trajectory_publisher = JointTrajectoryPublisher()
    rclpy.spin(joint_trajectory_publisher)

    # 关闭节点
    joint_trajectory_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
