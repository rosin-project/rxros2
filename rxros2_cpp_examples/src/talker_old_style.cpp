/*
Copyright (c) 2020, ROSIN-project

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/


#include <rxros2/rxros2.h>
#include "std_msgs/msg/string.hpp"
using namespace rxcpp::operators;
using namespace rxros2::operators;

static std::string hello = "hello world ";

static std_msgs::msg::String mk_msg (const std::string& s) {
    std_msgs::msg::String msg;
    msg.data = s;
    return msg;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto talker = rxros2::create_node("talker");

    rxcpp::observable<>::interval (std::chrono::milliseconds (1000))
        | map ([&](int i) { return mk_msg(hello + std::to_string(i)); })
        | tap ([talker](const std_msgs::msg::String& msg) {
            RCLCPP_INFO(talker->get_logger(), "%s", msg.data.c_str()); })
        | sample_with_frequency(2.0)
        | publish_to_topic<std_msgs::msg::String>(talker, "/chatter", 1000);

    rclcpp::spin(talker);
    rclcpp::shutdown();
    return 0;
}
