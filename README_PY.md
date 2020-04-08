# RxROS2 - Reactive Programming for ROS2

## Introduction

RxROS2 is new API for ROS2 based on the paradigm of reactive programming. Reactive programming is an alternative to callback-based programming for implementing concurrent message passing systems that emphasizes explicit data flow over control flow. It makes the message flow and transformations of messages easy to capture in one place. It eliminates problems with deadlocks and understanding how callbacks interact. It helps you to make your nodes functional, concise, and testable. RxROS2 aspires to the slogan ‘concurrency made easy’.

## Contents

* [RxROS2 - reactive Programming for ROS2](#rxros2)
   * [Introduction](#introduction)
   * [Acknowledgement](#acknowledgement)
   * [Example Package](#example-package)
   * [Setup and Installation](#setup-and-installation)
   * [Creating a RXOS2 Node](#creating-a-rxros2-node)
       * [Creating a RxROS2 Node using a class](#creating-a-rxros2-node-using-a-class)
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
      * [Sample with Frequency](#sample-with-frequency)
         * [Syntax:](#syntax-3)
         * [Example:](#example-5)
   * [Example 1: A Keyboard Publisher](#example-1-a-keyboard-publisher)
   * [Example 2: A Velocity Publisher](#example-2-a-velocity-publisher)

## Acknowledgement

The RxROS2 library depends on and uses the following software:<br>

1. Ubuntu Bionic 18.04<br>
2. ROS2 Eloquent Elusor<br>
3. Reactive Python, RxPY v3.0.1<br>
https://github.com/ReactiveX/RxPY<br>
Released under the MIT license<br>

## Example Package

TBD

## Setup and Installation

TBD

## Creating a RxROS2 Node

A RxROS2 node is fundamentally a ROS2 node. It can be created in two distinct ways: Either by means of creating a class that is a sub-class of a `rxros2.Node` or by using the function `rxros2.create_node`.

### Creating a RxROS2 Node using a class

The following code shows how a RxROS2 node is created using a class:

```python
import rclpy
import rxros2

class MyNode(rxros2.Node):
    def __init__(self):
        super().__init__("my_node")

    def run(self):
        # add your code here ...

def main(args=None):
    rclpy.init(args=args)
    my_node = MyNode()
    my_node.start()
    rclpy.spin(my_node)
    my_node.destroy_node()
    rclpy.shutdown()
```

Common for all RxROS2 programs is that they import the `rxros2` library. It contains all the necessary code, including observables and operators, to get started using reactive programming (RxPY) with ROS2.

MyNode is further defined as a  `rxros2.Node`. The `rxros2.Node` is a very simple class. It is a sub-class of `rclpy.node.Node` and therefore also a ROS2 node. The constructor of the `rxros2.Node` takes the name of the node as argument. In this case "my_node". The `rxros2.Node` is an abstract class with one abstract method named `run` that must be implemented by the sub-class. `rxros2.Node` contains further a function named `start`. It will execute the `run` function in a new thread.

The main function is straight forward: It first initialize rclpy. Then it creates an instance of MyNode and executes the `start` function. The `start` function will execute the `run` function of MyNode in a new thread. It is possible to call `run` directly from the main function simply by executing `my_node.run()`. But be sure in this case that the `run` function is not blocking or else the `rclpy.spin` function is not called. `rclpy.spin` is needed in order to publish and listen to ROS2 topics. `rclpy.spin` is a blocking function that will continue to run until it is stopped/terminated. In this case `my_node.destry_node` and `rclpy.shutdown` are called to terminate the node.

### Creating a RxROS2 Node using the create_node function

The other other way to create a RxROS2 node is by using the function call `rxros2.create_node`. This is done as follows:

```python
import rclpy
import rxros2

def main(args=None):
    rclpy.init(args=args)
    my_node = rxros2.create_node("my_node")

    # add your code here ...

    rclpy.spin(my_node)
    my_node.destroy_node()
    rclpy.shutdown()
```

The `rxros2.create_node` takes the node name argument and the created node can be given directly as argument to the `rclpy.spin` function.

## Observables

Observables are asynchronous message streams. They are the fundamental data structure used by RxROS2. As soon as we have the observables RxROS2 and Rxpython will provide us with a number of functions and operators to manipulate the streams.

### Observable from a Topic

An observable data stream is created from a topic simply by calling the `rxros2.from_topic` function. The function takes four arguments, a node, the topic type, the name of the topic and an optional queue size.

The example below demonstrates how two ROS2 topics named “/joystick” and “/keyboard” are turned into two observable streams by means of the `rxros2.from_topic` function and then merged together into a new observable message stream named `teleop_obsrv`. Observe the use of the map operator: Since `teleop_msgs.Joystick` and `teleop_msgs.Keyboard` are different message types it is not possible to merge them directly. The map operator solves this problem by converting each `teleop_msgs.Joystick` and `teleop_msgs.Keyboard` message into a simple integer that represents the low level event of moving the joystick or pressing the keyboard.

#### Syntax

```python
def rxros2.from_topic(node: rclpy.node.Node, topic_type: Any, topic_name: str, queue_size=10) -> Observable:
```

#### Example

```python
import rclpy
import rxros2

class VelocityPublisher(rxros2.Node):
    def __init__(self):
        super().__init__("velocity_publisher")

    def run(self):
        joy_obsrv = rxros2.from_topic(self, teleop_msgs.Joystick, "/joystick").pipe(
            rxros2.map(lambda joy: return joy.event))
        key_obsrv = rxros2.from_topic(self, teleop_msgs.Keyboard, "/keyboard").pipe(
            rxros2.map(lambda key: return key.event))
        teleop_obsrv = joyObsrv.pipe(merge(key_obsrv))
        # ...

def main(args=None):
    rclpy.init(args=args)
    velocity_publisher = VelocityPublisher()
    velocity_publisher.start()
    rclpy.spin(velocity_publisher)
    velocity_publisher.destroy_node()
    rclpy.shutdown()
```

#### Example

```python
import rclpy
import rxros2

def main(args=None):
    rclpy.init(args=args)
    velocity_publisher = rxros2.create_node("velocity_publisher")

    joy_obsrv = rxros2.from_topic(velocity_publisher, teleop_msgs.Joystick, "/joystick").pipe(
        rxros2.map(lambda joy: return joy.event))
    key_obsrv = rxros2.from_topic(velocity_publisher, teleop_msgs.Keyboard, "/keyboard").pipe(
        rxros2.map(lambda key: return key.event))
    teleop_obsrv = joyObsrv.pipe(rxros2.merge(key_obsrv))
    #...

    rclpy.spin(velocity_publisher)
    velocity_publisher.destroy_node()
    rclpy.shutdown()
}
```

### Observable from a Linux device

The function `rxros2.from_device` will turn a Linux block or character device like `/dev/input/js0` into an observable message stream. `rxros2.from_device` has as such nothing to do with ROS2, but it provides an interface to low-level data types that are needed in order to create e.g. keyboard and joystick observables. The `rxros2.from_device` takes as argument the name of the device and a type of the data that are read from the device.

The example below shows what it takes to turn a stream of low-level joystick events into an observable message stream and publish them on a ROS2 topic. First an observable message stream is created from the device `/dev/input/js0`. Then is is converted it to a stream of ROS2 messages and finally the messages are published to a ROS2 topic. Three simple steps, that’s it!

#### Syntax

```python
def rxros2.from_device(device_name: str, struct_format: str) -> Observable:
```

#### Example

```python
def main(args=None):
    rclpy.init(args=args)
    joystick_publisher = rxros2.create_node("joystick_publisher");

    rxros2.from_device("llHHI", "/dev/input/js0").pipe(
        rxros2.map(joystickEvent2JoystickMsg),
        rxros2.publish_to_topic(joystick_publisher, teleop_msgs.Joystick "/joystick"))
    #...

    rclpy.spin(joystick_publisher)
    joystick_publisher.destroy_node()
    rclpy.shutdown()
}
```

### Operators

One of the primary advantages of stream oriented processing is the fact that we can apply functional programming primitives on them. Rxpython operators are nothing but filters, transformations, aggregations and reductions of the observable message streams we created in the previous section.

#### Publish to Topic

`rxros2::operators::publish_to_topic` is a rather special operator. It does not modify the message steam - it is in other words an identity function/operator. It will however take each message from the stream and publish it to a specific topic. This means that it is perfectly possible to continue modifying the message stream after it has been published to a topic.

##### Syntax:

```python
def to_topic(node: rclpy.node.Node, topic_type: Any, topic_name: str, queue_size=10) -> Callable[[Observable], Observable]
```

##### Example:

```python
class JoystickPublisher(rxros2.Node):
    def __init__(self):
        super().__init__("joystick_publisher")

    def run(self):
        rxros2.from_device("llHHI", "/dev/input/js0").pipe(
            rxros2.map(joystickEvent2JoystickMsg),
            rxros2.publish_to_topic(joystick_publisher, teleop_msgs.Joystick "/joystick"))
        #...


def main(args=None):
    rclpy.init(args=args)
    joystick_publisher = JoystickPublisher()
    talker.start()
    rclpy.spin(joystick_publisher)
    joystick_publisher.destroy_node()
    rclpy.shutdown()
```

### Sample with Frequency

The operator `rxros2.sample_with_frequency` will at regular intervals emit the last element or message of the observable message stream it was applied on - that is independent of whether it has changed or not. This means that the observable message stream produced by `rxros2.sample_with_frequency` may contain duplicated messages if the frequency is too high and it may miss messages in case the frequency is too low. This is the preferred way in ROS2 to publish messages on a topic and therefore a needed operation.

The operation operator `rxros2.sample_with_frequency` comes in two variants. One that is executing in the current thread and one that is executing in a specified thread also known as a coordination in Rxpython.

#### Syntax:

```python
def rxros2.sample_with_frequency(frequency: float) -> Callable[[Observable], Observable]:
```

#### Example:

```python
def main(args=None):
    rclpy.init(args=args)
    joystick_publisher = rxros2.create_node("joystick_publisher");

    rxros2.from_device("llHHI", "/dev/input/js0").pipe(
        rxros2.map(joystickEvent2JoystickMsg),
        rxros2.sample_with_frequency(frequencyInHz),
        rxros2.publish_to_topic(joystick_publisher, teleop_msgs.Joystick "/joystick"))
    #...

    rclpy.spin(joystick_publisher)
    joystick_publisher.destroy_node()
    rclpy.shutdown()
}
```

## Example 1: A Keyboard Publisher

The following example is a full implementation of a keyboard publisher that takes input from a Linux block device and publishes the low-level keyboard events to a ROS2 topic '/keyboard'.

```python
```

## Example 2: A Velocity Publisher

The following example is a full implementation of a velocity publisher
that takes input from a keyboard and joystick and publishes Twist messages
on the /cmd_vel topic.

```python
}
```
