import rclpy
from time import sleep
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

joint_names = [
    "ankle_p_joint",
    "knee_p_joint",
    "waist1_p_joint",
    "waist2_y_joint",
    "head1_y_joint",
    "head2_p_joint",
    "l_arm1_shoulder_p_joint",
    "l_arm2_shoulder_r_joint",
    "l_arm3_shoulder_y_joint",
    "l_arm4_elbow_p_joint",
    "l_arm5_wrist_y_joint",
    "l_arm6_wrist_p_joint",
    "l_arm7_wrist_r_joint",
    "r_arm1_shoulder_p_joint",
    "r_arm2_shoulder_r_joint",
    "r_arm3_shoulder_y_joint",
    "r_arm4_elbow_p_joint",
    "r_arm5_wrist_y_joint",
    "r_arm6_wrist_p_joint",
    "r_arm7_wrist_r_joint",
    "l_thumb_cmc_y_joint",
    "l_thumb_cmc_p_joint",
    "l_index_mcp_joint",
    "l_middle_mcp_joint",
    "l_ring_mcp_joint",
    "l_pinky_mcp_joint",
    "r_thumb_cmc_y_joint",
    "r_thumb_cmc_p_joint",
    "r_index_mcp_joint",
    "r_middle_mcp_joint",
    "r_ring_mcp_joint",
    "r_pinky_mcp_joint"
]


class JointTrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('joint_trajectory_controller_demo')
        self.publisher_ = self.create_publisher(JointTrajectory, '/body_controller/joint_trajectory', 10)
        self.timer = self.create_timer(1, self.timer_cb)

    def timer_cb(self):
        duration = 5
        self.publish_joint_trajectory(0.1, duration)
        sleep(duration + 1)

        duration = 10
        self.publish_joint_trajectory(-0.1, duration)
        sleep(duration + 1)

        duration = 5
        self.publish_joint_trajectory(0.0, duration)
        sleep(duration + 1)

        self.timer.cancel()
        exit(0)

    def publish_joint_trajectory(self, position, duration):
        msg = JointTrajectory()
        msg.joint_names = joint_names

        point = JointTrajectoryPoint()
        for joint_name in msg.joint_names:
            point.positions.append(position)

        point.time_from_start.sec = duration

        msg.points = [point]

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing JointTrajectory with {position} positions')


def demo_topic_main(args=None):
    rclpy.init(args=args)

    joint_trajectory_publisher = JointTrajectoryPublisher()
    rclpy.spin(joint_trajectory_publisher)

    joint_trajectory_publisher.destroy_node()
    rclpy.shutdown()


class FollowJointTrajectoryClient(Node):
    def __init__(self):
        super().__init__('follow_joint_trajectory_client')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/body_controller/follow_joint_trajectory')

    def send_goal(self, joint_names, points):
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available!')
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = points

        self.get_logger().info('Sending goal...')
        self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback).add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {[round(point, 2) for point in feedback.actual.positions]}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Action result code: {result.error_code}, string: {result.error_string}')
        rclpy.shutdown()


def demo_action_client_main(args=None):
    rclpy.init(args=args)

    client = FollowJointTrajectoryClient()
    points = []

    point = JointTrajectoryPoint()
    for joint in joint_names:
        point.positions.append(0.1)
    point.time_from_start.sec = 5
    points.append(point)

    point = JointTrajectoryPoint()
    for joint in joint_names:
        point.positions.append(-0.1)
    point.time_from_start.sec = 15
    points.append(point)

    point = JointTrajectoryPoint()
    for joint in joint_names:
        point.positions.append(0.0)
    point.time_from_start.sec = 20
    points.append(point)

    client.send_goal(joint_names, points)

    rclpy.spin(client)


if __name__ == '__main__':
    # demo_topic_main()
    demo_action_client_main()
