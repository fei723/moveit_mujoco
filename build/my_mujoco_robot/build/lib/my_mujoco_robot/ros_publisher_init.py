
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class CtrSignalPublisher(Node):
    def __init__(self):
        super().__init__('ctr_signal_publisher')
        # 创建一个发布者，发布 Float64MultiArray 类型的消息到 'ctr_signal_topic' 话题，队列大小为 10
        self.publisher_ = self.create_publisher(Float64MultiArray, 'ctr_signal_topic', 10)

    def publish_ctr_signal(self, ctr_signal):
        # 创建消息对象
        msg = Float64MultiArray()
        # 将 ctr_signal 数组转换为列表并赋值给消息的数据字段
        msg.data = ctr_signal.tolist()
        # 发布消息
        self.publisher_.publish(msg)
        # 打印日志信息
        self.get_logger().info('Publishing control signal: "%s"' % str(msg.data))

class StatePublisher(Node):
    def __init__(self):
        super().__init__('state_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray,'state_topic', 10)

    def publish_state(self, state):
        msg = Float64MultiArray()
        msg.data = state.tolist()
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing state: "%s"' % str(msg.data))