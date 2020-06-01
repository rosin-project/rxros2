import rclpy
from rxros2_pytest2 import rxros2
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


class T2(rxros2.Node):
    def __init__(self):
        super().__init__("T2")
        self.bulb_data = ''.rjust(1048576, '#')
        self.get_logger().info('Starting RxPyNode T2:')

    def run(self):
        rxros2.from_topic(self, Test, "/T1T").subscribe(
            lambda msg: self.get_logger().info('%d,%d,%d' % (msg.msg_no, len(msg.data), age_of(msg.time_stamp))))

        rxros2.interval(0.05).pipe(
            rxros2.map(lambda msg_no: mk_test_msg(msg_no, self.bulb_data)),
            rxros2.publish_to_topic(self, Test, "/T2T"))


def main(args=None):
    rclpy.init(args=args)
    t2 = T2()
    t2.start()
    rclpy.spin(t2)
    t2.destroy_node()
    rclpy.shutdown()
