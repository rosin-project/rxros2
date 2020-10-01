# RxROS2 - Reactive Programming for ROS2

## Introduction

RxROS2 is new API for ROS2 based on the paradigm of reactive programming. Reactive programming is an alternative to callback-based programming for implementing concurrent message passing systems that emphasizes explicit data flow over control flow. It makes the message flow and transformations of messages easy to capture in one place. It eliminates problems with deadlocks and understanding how callbacks interact. It helps you to make your nodes functional, concise, and testable. RxROS2 aspires to the slogan ‘concurrency made easy’.

## Contents

   * [RxROS2 - reactive Programming for ROS2](#rxros2)
      * [Introduction](#introduction)
      * [Acknowledgement](#acknowledgement)
      * [Creating a RxROS2 Node](#creating-a-rxros2-node)
          * [Creating a RxROS2 Node using a Class](#creating-a-rxros2-node-using-a-class)
          * [Creating a RxROS2 Node using the create_node function](#creating-a-rxros2-node-using-the-create_node-function)
      * [Observables](#observables)
         * [Observable from a Topic](#observable-from-a-topic)
            * [Syntax](#syntax)
            * [Example](#example)
            * [Example](#example-2)
         * [Observable from a Linux device](#observable-from-a-linux-device)
            * [Syntax](#syntax-1)
            * [Example](#example-3)
      * [Operators](#operators)
         * [Publish to Topic](#publish-to-topic)
            * [Syntax:](#syntax-2)
            * [Example:](#example-4)
         * [Call Service](#call-service)
            * [Syntax:](#syntax-3)
         * [Sample with Frequency](#sample-with-frequency)
            * [Syntax:](#syntax-4)
            * [Example:](#example-5)
      * [Example 1: A Keyboard Publisher](#example-1-a-keyboard-publisher)
      * [Example 2: A Velocity Publisher](#example-2-a-velocity-publisher)
      * [Performance Measurements](#performance-measurements)
         * [Test setup](#test-setup)
         * [Test programs](#test-programs)
         * [Test results](#test-results)

## Acknowledgement
This projects has received funding from the European Union's Horizon 2020 research and innovation programme under grant agreement No 732287.

![](https://rosin-project.eu/wp-content/uploads/2017/03/EU-Flag-1.png)<br>
[https://rosin-project.eu](https://rosin-project.eu)

## Creating a RxROS2 Node

A RxROS2 node is fundamentally a ROS2 node. It can be created in two distinct ways: Either by means of creating a class that is a sub-class of a `rxros2::Node` or by using the function `rxros2::create_node`.

### Creating a RxROS2 Node using a Class

The following code shows how a RxROS2 node is created using a class:

```cpp
#include <rxros/rxros2.h>

struct MyNode: public rxros2::Node {
    MyNode(): rxros2::Node("my_node") {}

    void run() {
      //... add your code here
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto my_node = std::make_shared<MyNode>();
    my_node->start();
    rclcpp::spin(my_node);
    rclcpp::shutdown();
    return 0;
}
```

Common for all RxROS2 programs is that they include the `rxros/rxros2.h` header file. It contains all the necessary code, including observables and operators, to get started using reactive programming (RxCpp) with ROS2.


MyNode is defined as `struct` rather than a `class` to take advantage of that all properties and member functions are public. MyNode is further defined as a  `rxros2::Node`. The `rxros2::Node` is a very simple class. It is a sub-class of `rclcpp::Node` and therefore also a ROS2 node. The constructor of the `rxros2::Node` takes the name of the node as argument. In this case "my_node". The `rxros2::Node` has a virtual method named `run` that must be implemented by the sub-class, i.e. MyNode in this case. `rxros2::Node` contains further a function `start`. It will execute the `run` function in a new thread.

The main function is straight forward: It first initialize rclcpp. Then it creates an instance of MyNode and executes the `start` function. The `start` function will execute the `run` function of MyNode in a new thread. It is possible to call `run` directly from the main function simply by executing `my_node->run()`. But be sure in this case that the `run` function is not blocking or else the `rclcpp::spin` function is not called. `rclcpp::spin` is needed in order to publish and listen to ROS2 topics. `rclcpp::shutdown` is finally called to terminate the node in case the `rclcpp::spin` function has been terminated.

### Creating a RxROS2 Node using the create_node function

The other other way to create a RxROS2 node is by using the function call `rxros2::create_node`. This is done as follows:

```cpp
#include <rxros/rxros2.h>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto my_node = rxros2::create_node("my_node");

    // ... add your code here

    rclcpp::spin(my_node);
    rclcpp::shutdown();
    return 0;
}
```

The `rxros2::create_node` takes the node name argument and the created node can be given directly as argument to the `rclcpp::spin` function.

## Observables

Observables are asynchronous message streams. They are the fundamental data structure used by RxROS2. As soon as we have the observables RxROS2 and RxCpp will provide us with a number of functions and operators to manipulate the streams.

### Observable from a Topic

An observable data stream is created from a topic by calling the `rxros2::observable::from_topic` function. The function takes three arguments, a node, the name of the topic and an optional quality of service for the ROS2 topic. In order to use the `rxros2::observable::from_topic` function it is important also to specify the type of the topic messages.

The example below demonstrates how two ROS2 topics named “/joystick” and “/keyboard” are turned into two observable streams by means of the `rxros2::observable::from_topic` function and then merged together into a new observable message stream named `teleop_obsrv`. Observe the use of the map operator: Since `teleop_msgs::msg::Joystick` and `teleop_msgs::msg::Keyboard` are different message types it is not possible to merge them directly. The map operator solves this problem by converting each `teleop_msgs::::msg:Joystick` and `teleop_msgs::msg::Keyboard` message into a simple integer that represents the low level event of moving the joystick or pressing the keyboard.

The pipe operator “|” is a specialty of RxCpp that is used as a simple mechanism to compose operations on observable message streams. The usual “.” notation could have been used just as well, but it’s common to use the pipe operator “|” in RxCpp.

#### Syntax

```cpp
auto rxros2::observable::from_topic<topic_type>(rxros2::Node* node, const std::string& topic_name, const rclcpp::QoS& qos = 10);
auto rxros2::observable::from_topic<topic_type>(std::shared_ptr<rxros2::Node> node, const std::string& topic_name, const rclcpp::QoS& qos = 10);
```

#### Example

```cpp
struct VelocityPublisher: public rxros2::Node {
    MyNode(): rxros2::Node("velocity_publisher") {}

    void run() {
      auto joy_obsrv = rxros2::observable::from_topic<teleop_msgs::msg::Joystick>(this, "/joystick")
          | map([](teleop_msgs::msg::Joystick joy) { return joy.event; });
      auto key_obsrv = rxros2::observable::from_topic<teleop_msgs::msg::Keyboard>(this, "/keyboard")
          | map([](teleop_msgs::msg::Keyboard key) { return key.event; });
      auto teleop_obsrv = joyObsrv.merge(keyObsrv);
      //...
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto velocity_publisher = std::make_shared<VelocityPublisher>();
    velocity_publisher->start();
    rclcpp::spin(velocity_publisher);
    rclcpp::shutdown();
    return 0;
}
```

#### Example

```cpp
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto velocity_publisher = rxros2::create_node("velocity_publisher");

    auto joy_obsrv = rxros2::observable::from_topic<teleop_msgs::Joystick>(velocity_publisher, "/joystick")
        | map([](teleop_msgs::msg::Joystick joy) { return joy.event; });
    auto key_obsrv = rxros2::observable::from_topic<teleop_msgs::Keyboard>(velocity_publisher, "/keyboard")
        | map([](teleop_msgs::msg::Keyboard key) { return key.event; });
    auto teleop_obsrv = joyObsrv.merge(keyObsrv);
    //...

    rclcpp::spin(velocity_publisher);
    rclcpp::shutdown();
    return 0;
}
```

### Observable from a Linux device

The function `rxros2::observable::from_device` will turn a Linux block or character device like `/dev/input/js0` into an observable message stream. `rxros2::observable::from_device` has as such nothing to do with ROS2, but it provides an interface to low-level data types that are needed in order to create e.g. keyboard and joystick observables. The `rxros2::observable::from_device` takes as argument the name of the device and a type of the data that are read from the device.

The example below shows what it takes to turn a stream of low-level joystick events into an observable message stream and publish them on a ROS2 topic. First an observable message stream is created from the device `/dev/input/js0`. Then is is converted it to a stream of ROS2 messages and finally the messages are published to a ROS2 topic. Three simple steps, that’s it!

#### Syntax

```cpp
auto rxros2::observable::from_device<device_type>(const std::string& device_name);
```

#### Example

```cpp
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto joystick_publisher = rxros2::create_node("joystick_publisher");
    //...
    rxros2::observable::from_device<joystick_event>("/dev/input/js0")
        | map(joystickEvent2JoystickMsg)
        | publish_to_topic<teleop_msgs::msg::Joystick>(joystick_publisher, "/joystick");
    //...
    rclcpp::spin(velocity_publisher);
    rclcpp::shutdown();
    return 0;
}
```

### Operators

One of the primary advantages of stream oriented processing is the fact that we can apply functional programming primitives on them. RxCpp operators are nothing but filters, transformations, aggregations and reductions of the observable message streams we created in the previous section.

#### Publish to Topic

`rxros2::operators::publish_to_topic` is a rather special operator. It does not modify the message steam - it is in other words an identity function/operator. It will however take each message from the stream and publish it to a specific topic. This means that it is perfectly possible to continue modifying the message stream after it has been published to a topic.

##### Syntax:

```cpp
auto rxros2::operators::publish_to_topic<topic_type>(rxros2::Node* node, const std::string& topic_name, const rclcpp::QoS& qos = 10);
auto rxros2::operators::publish_to_topic<topic_type>(std::shared_ptr<rxros2::Node> node, const std::string& topic_name, const rclcpp::QoS& qos = 10);
```

##### Example:

```cpp
struct JoystickPublisher: public rxros2::Node {
    JoystickPublisher(): rxros2::Node("joystick_publisher") {}

    void run() {
      rxros2::observable::from_device<joystick_event>("/dev/input/js0")
          | map(joystickEvent2JoystickMsg)
          | publish_to_topic<teleop_msgs::msg::Joystick>(joystick_publisher, "/joystick");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto joystick_publisher = std::make_shared<JoystickPublisher>();
    joystick_publisher->start();
    rclcpp::spin(joystick_publisher);
    rclcpp::shutdown();
    return 0;
}
```

### Send request

Besides the publish/subscribe model, ROS2 also provides a request/reply model that allows a message to be send from one node (request) and handled by another node (reply) - it is a typical client-server mechanism that can be useful in distributed systems.

RxROS2 only provides a means to send a request, i.e. the client side. The server side will have to be created exactly the same way as it is done it ROS2. To send a request the `rxros2::operators::send_request` operator is called. It take a node and a service name as argument and a service type that specifies the type of the observable message stream the operation is applied on. The service type consists of a request and response part. The request part must be filled out prior to the send request call and the result will be a new observable stream where the response part has been filled out by the server part.

#### Syntax:

```cpp
auto send_request<service_type>(rxros2::Node* node, const std::string& service_name)
auto send_request<service_type>(std::shared_ptr<rxros2::Node> node, const std::string& service_name)
```

### Sample with Frequency

The operator `rxros2::operators::sample_with_frequency` will at regular intervals emit the last element or message of the observable message stream it was applied on - that is independent of whether it has changed or not. This means that the observable message stream produced by `rxros2::operators::sample_with_frequency` may contain duplicated messages if the frequency is too high and it may miss messages in case the frequency is too low. This is the preferred way in ROS2 to publish messages on a topic and therefore a needed operation.

The operation operator `rxros2::operators::sample_with_frequency` comes in two variants. One that is executing in the current thread and one that is executing in a specified thread also known as a coordination in RxCpp.

#### Syntax:

```cpp
auto rxros2::operators::sample_with_frequency(const double frequency);
auto rxros2::operation::sample_with_frequency(const double frequency, Coordination coordination);
```

#### Example:

```cpp
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto joystick_publisher = rxros2::create_node("joystick_publisher");
    //...
    | sample_with_frequency(frequencyInHz)
    | publish_to_topic<geometry_msgs::msg::Twist>(joystick_publisher, "/cmd_vel");
    //...
    rclcpp::spin(joystick_publisher);
    rclcpp::shutdown();
    return 0;
}
```

## Example 1: A Keyboard Publisher

The following example is a full implementation of a keyboard publisher that takes input from a Linux block device and publishes the low-level keyboard events to a ROS2 topic '/keyboard'.

```cpp
#include <rxros/rxros2.h>
#include <rxros2_teleop_msgs/Keyboard.h>
#include "KeyboardPublisher.h"
using namespace rxcpp::operators;
using namespace rxros2::operators;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto keyboard_publisher = rxros2::create_node("keyboard_publisher");

    std::string keyboardDevice;
    keyboard_publisher->declare_parameter("/keyboard_publisher/device", "/dev/input/event1");
    keyboard_publisher->get_parameter("/keyboard_publisher/device", keyboardDevice);

    auto keyboardEvent2KeyboardMsg = [](const auto keyboardEvent) {
        auto makeKeyboardMsg = [=] (auto event) {
            teleop_msgs::Keyboard keyboardMsg;
            keyboardMsg.time = ros::Time(keyboardEvent.time.tv_sec, keyboardEvent.time.tv_usec);
            keyboardMsg.event = event;
            return keyboardMsg;};
        if ((keyboardEvent.type == EV_KEY) && (keyboardEvent.value != REP_DELAY)) {
            if (keyboardEvent.code==KEY_UP)
                return makeKeyboardMsg(KB_EVENT_UP);
            else if (keyboardEvent.code==KEY_LEFT)
                return makeKeyboardMsg(KB_EVENT_LEFT);
            else if (keyboardEvent.code==KEY_RIGHT)
                return makeKeyboardMsg(KB_EVENT_RIGHT);
            else if (keyboardEvent.code==KEY_DOWN)
                return makeKeyboardMsg(KB_EVENT_DOWN);
            else if (keyboardEvent.code==KEY_SPACE)
                return makeKeyboardMsg(KB_EVENT_SPACE);
        }
        return makeKeyboardMsg(KB_EVENT_NONE);};

    rxros2::observable::from_device<input_event>(keyboardDevice)
        | map(keyboardEvent2KeyboardMsg)
        | publish_to_topic<teleop_msgs::msg::Keyboard>(keyboard_publisher, "/keyboard");

    rclcpp::spin(keyboard_publisher);
    rclcpp::shutdown();
    return 0;
}
```

## Example 2: A Velocity Publisher

The following example is a full implementation of a velocity publisher that takes input from a keyboard and joystick and publishes Twist messages on the /cmd_vel topic.

```cpp
#include <rxros/rxros2.h>
#include <rxros2_teleop_msgs/Joystick.h>
#include <rxros2_teleop_msgs/Keyboard.h>
#include <geometry_msgs/Twist.h>
#include "JoystickPublisher.h"
#include "KeyboardPublisher.h"
using namespace rxcpp::operators;
using namespace rxros2::operators;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto velocity_publisher = rxros2::create_node("velocity_publisher");

    float frequencyInHz;
    velocity_publisher->declare_parameter("/velocity_publisher/frequency", 10.0); /hz
    velocity_publisher->get_parameter("/velocity_publisher/frequency", );
    float min_vel_linear;
    velocity_publisher->declare_parameter("/velocity_publisher/min_vel_linear", 10.00); // m/s
    velocity_publisher->get_parameter("/velocity_publisher/min_vel_linear", );
    float max_vel_linear;
    velocity_publisher->declare_parameter("/velocity_publisher/max_vel_linear", 10.00); // m/s
    velocity_publisher->get_parameter("/velocity_publisher/max_vel_linear", );
    float min_vel_angular;
    velocity_publisher->declare_parameter("/velocity_publisher/min_vel_angular", 10.00); // rad/s
    velocity_publisher->get_parameter("/velocity_publisher/min_vel_angular", );
    float max_vel_angular;
    velocity_publisher->declare_parameter("/velocity_publisher/max_vel_angular", 10.00); // rad/s
    velocity_publisher->get_parameter("/velocity_publisher/max_vel_angular", );
    float deltaVelLinear = (maxVelLinear - minVelLinear) / 10.0;
    float deltaVelAngular = (maxVelAngular - minVelAngular) / 10.0;

    RCLCPP_INFO(velocity_publisher->get_logger(), "frequency: %f", frequencyInHz);
    RCLCPP_INFO(velocity_publisher->get_logger(), "min_vel_linear: %f m/s", minVelLinear);
    RCLCPP_INFO(velocity_publisher->get_logger(), "max_vel_linear: %f m/s", maxVelLinear);
    RCLCPP_INFO(velocity_publisher->get_logger(), "min_vel_angular: %f rad/s", minVelAngular);
    RCLCPP_INFO(velocity_publisher->get_logger(), "max_vel_angular: %f rad/s", maxVelAngular);

    auto adaptVelocity = [=] (auto newVel, auto minVel, auto maxVel, auto isIncrVel) {
        if (newVel > maxVel)
            return maxVel;
        else if (newVel < -maxVel)
            return -maxVel;
        else if (newVel > -minVel && newVel < minVel)
            return (isIncrVel) ? minVel : -minVel;
        else
            return newVel;};

    auto teleop2VelTuple = [=](const auto& prevVelTuple, const int event) {
        const auto prevVelLinear = std::get<0>(prevVelTuple);  // use previous linear and angular velocity
        const auto prevVelAngular = std::get<1>(prevVelTuple); // to calculate the new linear and angular velocity.
        if (event == JS_EVENT_BUTTON0_DOWN || event == JS_EVENT_BUTTON1_DOWN || event == KB_EVENT_SPACE)
            return std::make_tuple(0.0, 0.0); // Stop the robot
        else if (event == JS_EVENT_AXIS_UP || event == KB_EVENT_UP)
            return std::make_tuple(adaptVelocity((prevVelLinear + deltaVelLinear), minVelLinear, maxVelLinear, true), prevVelAngular); // move forward
        else if (event == JS_EVENT_AXIS_DOWN || event == KB_EVENT_DOWN)
            return std::make_tuple(adaptVelocity((prevVelLinear - deltaVelLinear), minVelLinear, maxVelLinear, false), prevVelAngular); // move backward
        else if (event == JS_EVENT_AXIS_LEFT || event == KB_EVENT_LEFT)
            return std::make_tuple(prevVelLinear, adaptVelocity((prevVelAngular + deltaVelAngular), minVelAngular, maxVelAngular, true)); // move left
        else if (event == JS_EVENT_AXIS_RIGHT || event == KB_EVENT_RIGHT)
            return std::make_tuple(prevVelLinear, adaptVelocity((prevVelAngular - deltaVelAngular), minVelAngular, maxVelAngular, false)); // move right
        else
            return std::make_tuple(prevVelLinear, prevVelAngular));}; // do nothing

    auto velTuple2TwistMsg = [](auto velTuple) {
        geometry_msgs::Twist vel;
        vel.linear.x = std::get<0>(velTuple);
        vel.angular.z = std::get<1>(velTuple);
        return vel;};

    auto joyObsrv = rxros2::Observable::fromTopic<teleop_msgs::msg::Joystick>(velocity_publisher, "/joystick") // create an Observable stream from "/joystick" topic
        | map([](teleop_msgs::Joystick joy) { return joy.event; });
    auto keyObsrv = rxros2::Observable::fromTopic<teleop_msgs::msg::Keyboard>(velocity_publisher, "/keyboard") // create an Observable stream from "/keyboard" topic
        | map([](teleop_msgs::Keyboard key) { return key.event; });
    joyObsrv.merge(keyObsrv)                                                           // merge the joystick and keyboard messages into an Observable teleop stream.
        | scan(std::make_tuple(0.0, 0.0), teleop2VelTuple)                             // turn the teleop stream into a linear and angular velocity stream.
        | map(velTuple2TwistMsg)                                                       // turn the linear and angular velocity stream into a Twist stream.
        | sample_with_frequency(frequencyInHz)                                         // take latest Twist msg and populate it with the specified frequency.
        | publish_to_topic<geometry_msgs::msg::Twist>(velocity_publisher, "/cmd_vel"); // publish the Twist messages to the topic "/cmd_vel"

    RCLCPP_INFO(velocity_publisher->get_logger(), "Spinning velocity_publisher ...");
    rclcpp::spin(velocity_publisher);
    rclcpp::shutdown();
    return 0;
}
```

## Performance Measurements

In this section we will take a closer look at the performance of RxROS2. The test will compare a minimal subscriber/publisher program written in RxROS2 and plain ROS2. The test consists of two nodes that publish data and subscribe to data from the other node. The data contains a timestamp that will be updated at the moment the data is published and the latency will be calculated at the moment the other node receives the data. Besides the latency, CPU load, Memory consumption and number of used treads will be measured. Several tests will be performed with various amounts of data.

### Test setup
The tests will be performed on a RaspberryPi 4B with 4 Mbytes of RAM. The following software has been installed on the machine:

* Ubuntu 18.04.4 server image.
* ROS2 Eloquent Elusor (desktop version)
* RxCpp v2
* Latest version of RxROS2

The software has been installed as is. There has been no changes to RaspberryPi 4B to boost performance, nor has there been installed a special software to boost or optimize performance. The Ubuntu 18.04.4 and ROS2 packages has been updated via the Ubuntu package storage to the latest version.

### Test programs
To measure CPU load and RAM consumption the Linux `top` command has been used. To measure the number of used threads the `/proc/<pid>/status` file has been used and to measure latency the following programs has been used. The first is a plain ROS2 program and the other is a similar program written in RxROS2:

```cpp
#include <cstdio>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2_msg/msg/test.hpp"
using namespace std::chrono_literals;

class T1 : public rclcpp::Node
{
private:
    rclcpp::Subscription<ros2_msg::msg::Test>::SharedPtr subscription;
    rclcpp::Publisher<ros2_msg::msg::Test>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
    size_t msg_no;
    const std::string bolb_data = std::string(1048576, '#');

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
    T1(): Node("T1"), msg_no(0) {
        RCLCPP_INFO(this->get_logger(), "Starting CppNode T1:");

        subscription = this->create_subscription<ros2_msg::msg::Test>( "/T2T", 10,
            [this](ros2_msg::msg::Test::UniquePtr msg) {
                RCLCPP_INFO(this->get_logger(), "%d,%d,%d", msg->msg_no, msg->data.length(), age_of(msg->time_stamp));
            });

        publisher = this->create_publisher<ros2_msg::msg::Test>("/T1T", 10);

        auto timer_callback = [this]() -> void {
            auto msg = mk_test_msg(this->msg_no++, this->bolb_data);
            this->publisher->publish(msg);
        };
        timer = this->create_wall_timer(50ms, timer_callback);
    }
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<T1>());
    rclcpp::shutdown();
    return 0;
}
```

And the equivalent program written in RxROS2:

```cpp
#include <cstdio>
#include <rxros/rxros2.h>
#include "std_msgs/msg/string.hpp"
#include "ros2_msg/msg/test.hpp"
using namespace rxcpp::operators;
using namespace rxros2::operators;

struct T2: public rxros2::Node {
    T2(): rxros2::Node("T2") {};
    const std::string bolb_data = std::string(1048576, '#');

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
        RCLCPP_INFO(this->get_logger(), "Starting RxCppNode T2:");

        rxros2::observable::from_topic<ros2_msg::msg::Test>(this, "/T1T")
            .subscribe ( [this] (const ros2_msg::msg::Test::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "%d,%d,%d", msg->msg_no, msg->data.length(), age_of(msg->time_stamp));});

        rxcpp::observable<>::interval (std::chrono::milliseconds(50))
            | map ([&](int i) { return mk_test_msg(i, bolb_data); })
            | publish_to_topic<ros2_msg::msg::Test> (this, "/T2T");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto t2 = std::make_shared<T2>();
    t2->start();
    rclcpp::spin(t2);
    rclcpp::shutdown();
    return 0;
}
```

### Test results
The test results for ROS2 and RxROS2 (C++) are shown in the following table:

|Test|CPU min %|CPU max %|MEM min %|MEM max %|THREADS|LATENCY min ms|LATENCY max ms|LATENCY avg ms|
|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
|ros2 100B 40Hz|5.0|5.3|0.4|0.4|7|0.2527|1.0918|0.3690|
|rxros2 100B 40Hz|5.3|5.6|0.4|0.5|8|0.3458|0.8041|0.3918|
||||||||||
|ros2 1K 40Hz|5.3|5.3|0.4|0.4|7|0.2208|0.6157|0.3843|
|rxros2 1K 40Hz|5.3|5.6|0.4|0.4|8|0.2353|0.7893|0.3904|
||||||||||
|ros2 1M 40Hz|36.4|37.1|0.6|0.6|7|5.5996|27.6453|11.8814|
|rxros2 1M 40Hz|33.1|33.1|0.6|0.7|8|4.9858|29.7769|10.8161|
||||||||||
|ros2 16M 4Hz|33.4|35.1|3.0|3.4|7|48.3979|634.0808|82.0909|
|rxros2 16M 4Hz|23.8|24.2|2.6|2.6|8|40.1948|97.8250|42.6458|

The table shows 8 distinct test. The table is organized in 4 groups that consist of a ROS2 test followed by a similar test performed by a RxROS2 program. Test "ros2 100B 40Hz" means we are running two plain ROS2 programs that each publish 100 Bytes of data and subscribes to the data published by the other node. The data is published with a rate of 20Hz per note, or 40Hz in total for the two nodes.

From the table it looks like RxROS2 is outperforming plain ROS2. The larger data messages we use the more distinct is the performance improvement of RxROS2. This is an unexpected result! The expected result is that ROS2 would slightly outperform a similar RxROS2 program due to the more abstract/high level nature of the RxROS2 operators. So what is causing this outcome? If we look at the plain ROS2 program we see that the spinner, subscriber and publisher is all working in the same thread. The RxROS2 program work in a slightly different manner. During startup of the node the node is actually started up in a new thread by the t2->start() command:

```cpp
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto t2 = std::make_shared<T2>();
    t2->start();
    rclcpp::spin(t2);
    rclcpp::shutdown();
    return 0;
}
```

This means that the `run` method will be called in a separate thread:

```cpp
void run() {
    RCLCPP_INFO(this->get_logger(), "Starting RxCppNode T2:");

    rxros2::observable::from_topic<ros2_msg::msg::Test>(this, "/T1T")
        .subscribe ( [this] (const ros2_msg::msg::Test::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "%d,%d,%d", msg->msg_no, msg->data.length(), age_of(msg->time_stamp));});

    rxcpp::observable<>::interval (std::chrono::milliseconds(50))
        | map ([&](int i) { return mk_test_msg(i, bolb_data); })
        | publish_to_topic<ros2_msg::msg::Test> (this, "/T2T");
}
```

The thread will however terminate very fast as the observable `from_topic` and `interval` are non-blocking. In fact, the `interval` observable is actually executing in a dedicated thread (scheduler), which means that topics will be published in a dedicated thread whereas the spinner, subscription and processing of topics are performed in the main thread. This is the main difference between the RxROS2 test program and the plain ROS2 program. The RxROS2 program uses one more thread than the plain ROS2 program to separate the publishing and subscription of topics.
