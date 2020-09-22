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

#include <rxros/rxros2.h>
#include "std_msgs/msg/string.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto listener = rxros2::create_node("listener");

    rxros2::observable::from_topic<std_msgs::msg::String>(listener, "/chatter", 1000)
        .subscribe ( [listener] (const std_msgs::msg::String::SharedPtr msg) {
            RCLCPP_INFO(listener->get_logger(), "I heard: '%s'", msg->data.c_str());});

    rclcpp::spin(listener);
    rclcpp::shutdown();
    return 0;
}
