import rclpy
from rxros2_pytest1 import rxros2
from rclpy.time import Time
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


class T1(rxros2.Node):
    def __init__(self):
        super().__init__("T1")
        self.bulb_data = ''.rjust(1048576, '#')
        self.get_logger().info('Starting RxPyNode T1:')

    def run(self):
        rxros2.from_topic(self, Test, "/T2T").subscribe(
            lambda msg: self.get_logger().info('%d,%d,%d' % (msg.msg_no, len(msg.data), age_of(msg.time_stamp))))

        rxros2.interval(0.05).pipe(
            rxros2.map(lambda msg_no: mk_test_msg(msg_no, self.bulb_data)),
            rxros2.publish_to_topic(self, Test, "/T1T"))


def main(args=None):
    rclpy.init(args=args)
    t1 = T1()
    t1.start()
    rclpy.spin(t1)
    t1.destroy_node()
    rclpy.shutdown()
