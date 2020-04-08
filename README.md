# RxROS2 - Reactive Programming for ROS 2

## Introduction

RxROS2 is new API for ROS 2 based on the paradigm of reactive programming. Reactive programming is an alternative to callback-based programming for implementing concurrent message passing systems that emphasizes explicit data flow over control flow. It makes the message flow and transformations of messages easy to capture in one place. It eliminates problems with deadlocks and understanding how callbacks interact. It helps you to make your nodes functional, concise, and testable. RxROS2 aspires to the slogan ‘concurrency made easy’.

## Contents

   * [RxROS2 - reactive Programming for ROS 2](#rxros2)
      * [Introduction](#introduction)
      * [Acknowledgement](#acknowledgement)
      * [Example Package](#example-package)
      * [Setup and Installation](#setup-and-installation)
      * [Creating a RxROS2 Node](#creating-a-rxros2-node)
          * [Creating a RXOS2 Node using a Class](#creating-a-rxros2-node-class)
          * [Creating a RXOS2 Node using the create_node Function](#creating-a-rxros2-node-using-function)
      * [Observables](#observables)
         * [Observable from a Topic](#observable-from-a-topic)
            * [Syntax](#syntax-1)
            * [Example](#example-1)
         * [Observable from a Linux device](#observable-from-a-linux-device)
            * [Syntax](#syntax-2)
            * [Example](#example-2)
      * [Operators](#operators)
         * [Publish to Topic](#publish-to-topic)
            * [Syntax:](#syntax-3)
            * [Example:](#example-3)
         * [Call Service](#call-service)
            * [Syntax:](#syntax-3)
         * [Sample with Frequency](#sample-with-frequency)
            * [Syntax:](#syntax-5)
            * [Example:](#example-5)
      * [Example 1: A Keyboard Publisher](#example-1-a-keyboard-publisher)
      * [Example 2: A Velocity Publisher](#example-2-a-velocity-publisher)

## Acknowledgement

The RxROS2 library depends on and uses the following software:<br>

1. Ubuntu Bionic 18.04<br>
2. ROS 2 Eloquent Elusor<br>
3. Reactive C++ v2<br>
https://github.com/ReactiveX/RxCpp<br>
Released under the Microsoft Open Source Code of Conduct.<br>
The RxCpp library (header files) is installed as part of installing RxROS2<br>

## Example Package

TBD

## Setup and Installation

TBD

## Creating a RXOS2 Node

A RxROS2 node is fundamentally a ROS 2 node. It can be created in two distinct ways: Either by means of creating a class that is a sub-class of a `rxros2::Node` or by using the function `rxros2::create_node`.

### Creating a RXOS2 Node using a class

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

Common for all RxROS2 programs is that they include the `rxros/rxros2.h` header file. It contains all the necessary code, including observables and operators, to get started using reactive programming (RxCpp) with ROS 2.


MyNode is defined as `struct` rather than a `class` to take advantage of that all properties and member functions are public. MyNode is further defined as a  `rxros2::Node`. The `rxros2::Node` is a very simple class. It is a sub-class of `rclcpp::Node` and therefore also a ROS 2 node. The constructor of the `rxros2::Node` takes the name of the node as argument. In this case "my_node". The `rxros2::Node` is an abstract class with one abstract method named `run` that must be implemented by the sub-class, i.e. MyNode in this case. `rxros2::Node` contains further a function `start`. It will execute the `run` function in a new thread.

The main function is straight forward: It first initialize rclcpp. Then it creates an instance of MyNode and executes the `start` function. The `start` function will execute the `run` function of MyNode in a new thread. It is possible to call `run` directly from the main function simply by executing `my_node->run()`. But be sure in this case that the `run` function is not blocking or else the `rclcpp::spin` function is not called. `rclcpp::spin` is needed in order to publish and listen to ROS 2 topics. `rclcpp::shutdown` is finally called to terminate the node in case the `rclcpp::spin` function has been terminated.

### Creating a RXOS2 Node using the create_node function

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

An observable data stream is created from a topic simply by calling the `rxros2::observable::from_topic` function. The function takes three arguments, a node, the name of the topic and an optional queue size. In order to use the `rxros2::observable::from_topic` function it is important also to specify the type of the topic messages.

The example below demonstrates how two ROS 2 topics named “/joystick” and “/keyboard” are turned into two observable streams by means of the `rxros2::observable::from_topic` function and then merged together into a new observable message stream named `teleop_obsrv`. Observe the use of the map operator: Since `teleop_msgs::Joystick` and `teleop_msgs::Keyboard` are different message types it is not possible to merge them directly. The map operator solves this problem by converting each `teleop_msgs::Joystick` and `teleop_msgs::Keyboard` message into a simple integer that represents the low level event of moving the joystick or pressing the keyboard.

The pipe operator “|” is a specialty of RxCpp that is used as a simple mechanism to compose operations on observable message streams. The usual “.” notation could have been used just as well, but it’s common to use the pipe operator “|” in RxCpp.

#### Syntax

```cpp
auto rxros2::observable::from_topic<topic_type>(rclcpp::Node* node, const std::string& topic_name, const uint32_t queue_size = 10);
auto rxros2::observable::from_topic<topic_type>(std::shared_ptr<rclcpp::Node> node, const std::string& topic_name, const uint32_t queue_size = 10);
```

#### Example 1

```cpp
struct VelocityPublisher: public rxros2::Node {
    MyNode(): rxros2::Node("velocity_publisher") {}

    void run() {
      auto joy_obsrv = rxros2::observable::from_topic<teleop_msgs::Joystick>(this, "/joystick")
          | map([](teleop_msgs::Joystick joy) { return joy.event; });
      auto key_obsrv = rxros2::observable::from_topic<teleop_msgs::Keyboard>(this, "/keyboard")
          | map([](teleop_msgs::Keyboard key) { return key.event; });
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

#### Example 2

```cpp
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto velocity_publisher = rxros2::create_node("velocity_publisher");

    auto joy_obsrv = rxros2::observable::from_topic<teleop_msgs::Joystick>(velocity_publisher, "/joystick")
        | map([](teleop_msgs::Joystick joy) { return joy.event; });
    auto key_obsrv = rxros2::observable::from_topic<teleop_msgs::Keyboard>(velocity_publisher, "/keyboard")
        | map([](teleop_msgs::Keyboard key) { return key.event; });
    auto teleop_obsrv = joyObsrv.merge(keyObsrv);
    //...

    rclcpp::spin(velocity_publisher);
    rclcpp::shutdown();
    return 0;
}
```

### Observable from a Linux device

The function `rxros2::observable::from_device` will turn a Linux block or character device like `/dev/input/js0` into an observable message stream. `rxros2::observable::from_device` has as such nothing to do with ROS 2, but it provides an interface to low-level data types that are needed in order to create e.g. keyboard and joystick observables. The `rxros2::observable::from_device` takes as argument the name of the device and a type of the data that are read from the device.

The example below shows what it takes to turn a stream of low-level joystick events into an observable message stream and publish them on a ROS 2 topic. First an observable message stream is created from the device `/dev/input/js0`. Then is is converted it to a stream of ROS 2 messages and finally the messages are published to a ROS2 topic. Three simple steps, that’s it!

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
        | publish_to_topic<teleop_msgs::Joystick>(joystick_publisher, "/joystick");
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
auto rxros2::operators::publish_to_topic<topic_type>(rclcpp::Node* node, const std::string& topic_name, const uint32_t queue_size = 10);
auto rxros2::operators::publish_to_topic<topic_type>(std::shared_ptr<rclcpp::Node> node, const std::string& topic_name, const uint32_t queue_size = 10);
```

##### Example:

```cpp
struct JoystickPublisher: public rxros2::Node {
    JoystickPublisher(): rxros2::Node("joystick_publisher") {}

    void run() {
      rxros2::observable::from_device<joystick_event>("/dev/input/js0")
          | map(joystickEvent2JoystickMsg)
          | publish_to_topic<teleop_msgs::Joystick>(joystick_publisher, "/joystick");
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

### Call Service

Besides the publish/subscribe model, ROS 2 also provides a request/reply model that allows a to be send from one node (request) and handled by another node (reply) - it is a typical client-server mechanism that can be useful in distributed systems.

RxROS2 only provides a means to send a request, i.e. the client side. The server side will have to be created exactly the same way as it is done it ROS 2. To send a request the `rxros2::operators::call_service` operator is called. It take a service name as argument and service type that specifies the type of the observable message stream the operation was applied on. The service type consists of a request and response part. The request part must be filled out prior to the service call and the result will be a new observable stream where the response part has been filled out by the server part.

#### Syntax:

```cpp
auto rxros2::operators::call_service<service_type>(const std::string& service_name);
```

### Sample with Frequency

The operator `rxros2::operators::sample_with_frequency` will at regular intervals emit the last element or message of the observable message stream it was applied on - that is independent of whether it has changed or not. This means that the observable message stream produced by `rxros2::operators::sample_with_frequency` may contain duplicated messages if the frequency is too high and it may miss messages in case the frequency is too low. This is the preferred way in ROS 2 to publish messages on a topic and therefore a needed operation.

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
    | publish_to_topic<geometry_msgs::Twist>(joystick_publisher, "/cmd_vel");
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
        | publish_to_topic<teleop_msgs::Keyboard>(keyboard_publisher, "/keyboard");

    rclcpp::spin(joystick_publisher);
    rclcpp::shutdown();
    return 0;
}

```

## Example 2: A Velocity Publisher

The following example is a full implementation of a velocity publisher
that takes input from a keyboard and joystick and publishes Twist messages
on the /cmd_vel topic.

```cpp
#include <rxros/rxros2.h>
#include <rxros2_teleop_msgs/Joystick.h>
#include <rxros2_teleop_msgs/Keyboard.h>
#include <geometry_msgs/Twist.h>
#include "JoystickPublisher.h"
#include "KeyboardPublisher.h"


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto keyboard_publisher = rxros2::create_node("velocity_publisher");

    const auto frequencyInHz = rxros2::Parameter::get("/velocity_publisher/frequency", 10.0); // hz
    const auto minVelLinear = rxros2::Parameter::get("/velocity_publisher/min_vel_linear", 0.04); // m/s
    const auto maxVelLinear = rxros2::Parameter::get("/velocity_publisher/max_vel_linear", 0.10); // m/s
    const auto minVelAngular = rxros2::Parameter::get("/velocity_publisher/min_vel_angular", 0.64); // rad/s
    const auto maxVelAngular = rxros2::Parameter::get("/velocity_publisher/max_vel_angular", 1.60); // rad/s
    const auto deltaVelLinear = (maxVelLinear - minVelLinear) / 10.0;
    const auto deltaVelAngular = (maxVelAngular - minVelAngular) / 10.0;

    rxros2::Logging().info() << "frequency: " << frequencyInHz;
    rxros2::Logging().info() << "min_vel_linear: " << minVelLinear << " m/s";
    rxros2::Logging().info() << "max_vel_linear: " << maxVelLinear << " m/s";
    rxros2::Logging().info() << "min_vel_angular: " << minVelAngular << " rad/s";
    rxros2::Logging().info() << "max_vel_angular: " << maxVelAngular << " rad/s";

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

    auto joyObsrv = rxros2::Observable::fromTopic<teleop_msgs::Joystick>(velocity_publisher, "/joystick") // create an Observable stream from "/joystick" topic
        | map([](teleop_msgs::Joystick joy) { return joy.event; });
    auto keyObsrv = rxros2::Observable::fromTopic<teleop_msgs::Keyboard>(velocity_publisher, "/keyboard") // create an Observable stream from "/keyboard" topic
        | map([](teleop_msgs::Keyboard key) { return key.event; });
    joyObsrv.merge(keyObsrv)                                                      // merge the joystick and keyboard messages into an Observable teleop stream.
        | scan(std::make_tuple(0.0, 0.0), teleop2VelTuple)                        // turn the teleop stream into a linear and angular velocity stream.
        | map(velTuple2TwistMsg)                                                  // turn the linear and angular velocity stream into a Twist stream.
        | sample_with_frequency(frequencyInHz)                                    // take latest Twist msg and populate it with the specified frequency.
        | publish_to_topic<geometry_msgs::Twist>(velocity_publisher, "/cmd_vel"); // publish the Twist messages to the topic "/cmd_vel"

    rxros2::Logging().info() << "Spinning velocity_publisher ...";
    rclcpp::spin(keyboard_publisher);
    rclcpp::shutdown();
    return 0;
}
```
