#include <cstdio>
#include <rxros/rxros2.h>
#include "std_msgs/msg/string.hpp"
#include "ros2_msg/msg/test.hpp"
using namespace rxcpp::operators;
using namespace rxros2::operators;

struct T1: public rxros2::Node {
    T1(): rxros2::Node("T1") {};
    const std::string bulb_data = std::string(1048576, '#');

    static auto age_of(const rclcpp::Time& time) {
        auto curr_time_nano = rclcpp::Clock().now().nanoseconds();
        auto time_nano = time.nanoseconds();
        return curr_time_nano-time_nano;
    }

    static auto mk_test_msg(const int msg_no, const std::string& data) {
        ros2_msg::msg::Test msg;
        msg.msg_no = msg_no;
        msg.data = data;
        msg.time_stamp = rclcpp::Clock().now();
        return msg;
    }

    void run() {
        RCLCPP_INFO(this->get_logger(), "Starting RxCppNode T1:");

        rxros2::observable::from_topic<ros2_msg::msg::Test>(this, "/T2T")
            .subscribe ( [this] (const ros2_msg::msg::Test::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "%d,%d,%d", msg->msg_no, msg->data.length(), age_of(msg->time_stamp));});

        rxcpp::observable<>::interval (std::chrono::milliseconds(50))
            | map ([&](int i) { return mk_test_msg(i, bulb_data); })
            | publish_to_topic<ros2_msg::msg::Test> (this, "/T1T");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto t1 = std::make_shared<T1>();
    t1->start();
    rclcpp::spin(t1);
    rclcpp::shutdown();
    return 0;
}
