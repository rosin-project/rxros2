import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from rclpy.clock import Clock
from ros2_msg.msg import Test


def age_of(time: Time):
    curr_time_nano = Clock().now().to_msg().nanosec
    time_nano = time.nanosec
    return curr_time_nano-time_nano


def mk_test_msg(msg_no: int, data: str) -> Test:
    msg = Test()
    msg.msg_no = msg_no
    msg.data = data
    msg.time_stamp = Clock().now().to_msg()
    return msg


class T2(Node):
    def __init__(self):
        super().__init__('T2')
        self.subscription = self.create_subscription(Test, '/T1T', self.subscription_callback, 10)
        self.publisher = self.create_publisher(Test, '/T2T', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.msg_no = 0
        self.bulb_data = ''.rjust(1048576, '#')
        self.get_logger().info('Starting PyNode T2:')

    def subscription_callback(self, msg: Test):
        self.get_logger().info('%d,%d,%d' % (msg.msg_no, len(msg.data), age_of(msg.time_stamp)))

    def timer_callback(self):
        msg = mk_test_msg(self.msg_no, self.bulb_data)
        self.publisher.publish(msg)
        self.msg_no += 1


def main(args=None):
    rclpy.init(args=args)
    t2 = T2()
    rclpy.spin(t2)
    t2.destroy_node()
    rclpy.shutdown()
