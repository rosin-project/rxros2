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
   * [Example 1: A Topic Listener](#example-1-a-topic-listener)
   * [Example 2: A Topic Publisher](#example-2-a-topic-publisher)

## Acknowledgement
This projects has received funding from the European Union's Horizon 2020 research and innovation programme under grant agreement No 732287.

![](https://rosin-project.eu/wp-content/uploads/2017/03/EU-Flag-1.png)<br>
[https://rosin-project.eu](https://rosin-project.eu)

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

The other way to create a RxROS2 node is by using the function call `rxros2.create_node`. This is done as follows:

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

Observables are asynchronous message streams. They are the fundamental data structure used by RxROS2. As soon as we have the observables RxROS2 and RxPy will provide us with a number of functions and operators to manipulate the streams.

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
        teleop_obsrv = rxros2.merge(joy_Obsrv, key_obsrv)
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
    teleop_obsrv = rxros2.merge(joy_obsrv, key_obsrv)
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

One of the primary advantages of stream oriented processing is the fact that we can apply functional programming primitives on them. RxPy operators are nothing but filters, transformations, aggregations and reductions of the observable message streams we created in the previous section.

#### Publish to Topic

`rxros2.publish_to_topic` is a rather special operator. It does not modify the message steam - it is in other words an identity function/operator. It will however take each message from the stream and publish it to a specific topic. This means that it is perfectly possible to continue modifying the message stream after it has been published to a topic.

##### Syntax:

```python
def publish_to_topic(node: rclpy.node.Node, topic_type: Any, topic_name: str, queue_size=10) -> Callable[[Observable], Observable]:
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

#### Send Request

Besides the publish/subscribe model, ROS2 also provides a request/reply model that allows a remote procedure call (RPC) to be send from one node (request) and handled by another node (reply) - it is a typical client/server mechanism that can be useful in distributed systems.

RxROS2 only provides a means to send a request, i.e. the client side. The server side will have to be created exactly the same way as it is done it ROS2. To send a request the rxros2.send_request operator is called. It take a node, a service type and a service name as argument. The service type consists of a request and response part. The request part must be filled out prior to the service call and the result part will be a new observable stream that is returned by the send_request operator.

##### Syntax:

```python
def send_request(node: rclpy.node.Node, service_type: Any, service_name: str) -> Callable[[Observable], Observable]:
```

##### Example: Service Client

```python
import rxros2
import rclpy
from example_interfaces.srv import AddTwoInts


def mk_request(a, b) -> AddTwoInts.Request:
    req = AddTwoInts.Request()
    req.a = a
    req.b = b
    return req


class ServiceClient(rxros2.Node):
    def __init__(self):
        super().__init__('service_client')

    def run(self):
        obs = rxros2.range(1, 10).pipe(
            rxros2.map(lambda i: mk_request(i, i+1)),
            rxros2.send_request(self, AddTwoInts, "add_two_ints"))
        obs.subscribe(on_next=lambda response: print("Response {0}".format(response.sum)))


def main(args=None):
    rclpy.init(args=args)
    service_client = ServiceClient()
    service_client.run()
    rclpy.spin(service_client)
    service_client.destroy_node()
    rclpy.shutdown()
```

##### Example: Service Server (plain ROS2 code)

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class ServiceServer(Node):
    def __init__(self):
        super().__init__('service_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response


def main(args=None):
    rclpy.init(args=args)
    service_server = ServiceServer()
    rclpy.spin(service_server)
    rclpy.shutdown()
```


### Sample with Frequency

The operator `rxros2.sample_with_frequency` will at regular intervals emit the last element or message of the observable message stream it was applied on - that is independent of whether it has changed or not. This means that the observable message stream produced by `rxros2.sample_with_frequency` may contain duplicated messages if the frequency is too high and it may miss messages in case the frequency is too low. This is the preferred way in ROS2 to publish messages on a topic and therefore a needed operation.

The operation operator `rxros2.sample_with_frequency` comes in two variants. One that is executing in the current thread and one that is executing in a specified thread also known as a coordination in RxPy.

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

## Example 1: A Topic Listener

The following example is a full implementation of a ROS2 topic listener using RxROS2. It comes in two flavors: One using a dedicated class implementation and one using the rxros2.create_node function

### Example 1a: A Topic Listener using a dedicated class implementation

```python
import rclpy
import rxros2
from std_msgs.msg import String


class Listener(rxros2.Node):
    def __init__(self):
        super().__init__("listener")

    def run(self):
        observable = rxros2.from_topic(self, String, "/chatter")
        observable.subscribe(
            lambda s: print("Received {0}".format(s.data)),
            lambda e: print("Error Occurred: {0}".format(e)),
            lambda: print("Done!"))


def main(args=None):
    rclpy.init(args=args)
    listener = Listener()
    listener.start()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()
```

### Example 1b: A Topic Listener using the rxros2.create_node function

```python
import rclpy
import rxros2
from std_msgs.msg import String


def main(args=None):
    rclpy.init(args=args)
    listener = rxros2.create_node("listener")

    observable = rxros2.from_topic(listener, String, "/chatter")
    observable.subscribe(
        lambda s: print("Received2 {0}".format(s.data)),
        lambda e: print("Error Occurred: {0}".format(e)),
        lambda: print("Done!"))

    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()

```

## Example 2: A Topic Publisher

The following example is a full implementation of a ROS2 topic publisher using RxROS2. It comes again in two flavors: One using a dedicated class implementation and one using the rxros2.create_node function

### Example 2a: A Topic Publisher using a dedicated class implementation

```python
import rclpy
import rxros2
from std_msgs.msg import String


def mk_msg(s) -> String:
    msg = String()
    msg.data = s
    return msg


class Publisher(rxros2.Node):
    def __init__(self):
        super().__init__("publisher")

    def run(self):
        rxros2.interval(1.0).pipe(
            rxros2.map(lambda i: mk_msg("Hello world " + str(i))),
            rxros2.do_action(lambda s: print("Send {0}".format(s.data))),
            rxros2.sample_with_frequency(2.0),
            rxros2.publish_to_topic(self, String, "/chatter"))


def main(args=None):
    rclpy.init(args=args)
    publisher = Publisher()
    publisher.start()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()
```

### Example 2b: A Topic Publisher using the rxros2.create_node function

```python
import rclpy
import rxros2
from std_msgs.msg import String


def mk_msg(s) -> String:
    msg = String()
    msg.data = s
    return msg


def main(args=None):
    rclpy.init(args=args)
    publisher = rxros2.create_node("publisher")

    rxros2.interval(1.0).pipe(
        rxros2.map(lambda i: mk_msg("Hello world " + str(i))),
        rxros2.do_action(lambda s: print("Send2 {0}".format(s.data))),
        rxros2.sample_with_frequency(2.0),
        rxros2.publish_to_topic(publisher, String, "/chatter"))

    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()
```
