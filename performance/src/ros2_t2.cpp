#include <cstdio>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2_msg/msg/test.hpp"
using namespace std::chrono_literals;

class T2 : public rclcpp::Node
{
private:
    rclcpp::Subscription<ros2_msg::msg::Test>::SharedPtr subscription;
    rclcpp::Publisher<ros2_msg::msg::Test>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
    size_t msg_no;
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

public:
    T2(): Node("T2"), msg_no(0) {
        RCLCPP_INFO(this->get_logger(), "Starting CppNode T2:");

        subscription = this->create_subscription<ros2_msg::msg::Test>( "/T1T", 10,
            [this](ros2_msg::msg::Test::UniquePtr msg) {
                RCLCPP_INFO(this->get_logger(), "%d,%d,%d", msg->msg_no, msg->data.length(), age_of(msg->time_stamp));
            });

        publisher = this->create_publisher<ros2_msg::msg::Test>("/T2T", 10);

        auto timer_callback = [this]() -> void {
            auto msg = mk_test_msg(this->msg_no++, this->bulb_data);
            this->publisher->publish(msg);
        };
        timer = this->create_wall_timer(50ms, timer_callback);
    }
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<T2>());
    rclcpp::shutdown();
    return 0;
}
