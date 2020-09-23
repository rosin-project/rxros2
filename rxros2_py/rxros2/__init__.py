# Copyright (c) 2020, ROSIN-project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import threading
import select
import struct
import rclpy
from abc import abstractmethod
from rx import *
from rx.operators import *
from rx.disposable import Disposable
from rclpy.node import Node as ROS2Node
from rclpy.action import ActionClient, ActionServer


class Node(ROS2Node):
    def __init__(self, node_name: str):
        """
        The constructor of a Node. It initialize the super class rclpy.node.Node and
        spawns a new thread that executes the 'run' method. The 'run' method is abstract
        and must therefore be implemented by a sub-class of Node.

        :param node_name: The name of the node.
        """
        super().__init__(node_name)

    @abstractmethod
    def run(self):
        pass

    def start(self):
        threading.Thread(target=self.run).start()


def create_node(node_name: str) -> ROS2Node:
    """
    create_node is a simple wrapper function that will create a new instance of a rclpy Node.

    :param node_name: The name of the node.
    :return: An instance of a newly created node.
    """
    return rclpy.create_node(node_name)


def from_topic(node: ROS2Node, topic_type: Any, topic_name: str, qos=10) -> Observable:
    """
    The from_topic function creates an observable data stream from a ROS2 topic.

    :param node: An instance of a Node.
    :param topic_type: The type of the topic and also the type of the observable data elements.
    :param topic_name: The name of the topic.
    :param qos: The quality of service associated to the ROS2 topic.
    :return An observable data stream from a specified topic.
    """
    def _subscribe(observer, scheduler=None) -> Disposable:
        node.create_subscription(topic_type, topic_name, lambda msg: observer.on_next(msg), qos)
        return observer
    return create(_subscribe)


def from_device(device_name: str, struct_format: str) -> Observable:
    """
    The from_topic function creates an observable data stream from a linux device driver (e.g. /dev/input/event1).

    :param device_name: The name of the Linux device driver (e.g /dev/input/event1 for keyboard events)
    :param struct_format: The format of the events of the specified device driver (e.g. 'llHHI')
    :return: An observable data of events from the specified device driver.
             The events can be unpacked means of the function struct.unpack(struct_format, event)
    """
    def _run(observer):
        fd = os.open(device_name, os.O_RDONLY | os.O_NONBLOCK)
        sz = struct.calcsize(struct_format)
        while rclpy.ok():
            select.select([fd], [], [])
            event = os.read(fd, sz)
            observer.on_next(event)
        os.close(fd)
        observer.on_completed()

    def _subscribe(observer, scheduler=None) -> Disposable:
        threading.Thread(_run(observer)).start()
        return observer

    return create(_subscribe)


def publish_to_topic(node: ROS2Node, topic_type: Any, topic_name: str, qos=10) -> Callable[[Observable], Observable]:
    """
    The to_topic operator will take each message from the stream and publish it to a specific ROS2 topic.

    :param node: An instance of a Node.
    :param topic_type: The type of the observable data elements.
    :param topic_name: The name of the ROS2 topic to publish the messages to.
    :param qos: The quality of service associated to ROS2 publisher.
    :return: The observable data stream it operates on, i.e. it is an identity operator.
    """
    def _publish_to_topic(source) -> Observable:
        publisher = node.create_publisher(topic_type, topic_name, qos)
        source.subscribe(on_next=lambda msg: publisher.publish(msg))
        return source
    return _publish_to_topic


def send_request(node: ROS2Node, service_type: Any, service_name: str) -> Callable[[Observable], Observable]:
    """
    The send_request operator will send a request to a service server which returns a result.

    :param node: An instance of a Node.
    :param service_type: The type of service. It consist of a request part and a result part.
    :param service_name: The name of the ROS2 service that will process the request and return a result.
    :return: A observable message stream of the returned results.
    """
    def _send_request(source) -> Observable:
        def _subscribe(observer, scheduler=None) -> Disposable:
            client = node.create_client(service_type, service_name)
            client.wait_for_service()
            source.subscribe(on_next=lambda request: observer.on_next(client.call(request)))
            return observer
        return create(_subscribe)
    return _send_request


def send_goal(node: ROS2Node, action_type: Any, action_name: str) -> Callable[[Observable], Observable]:
    """
    The send_goal operator will send a goal to a action server which returns a result.

    :param node: An instance of a Node.
    :param action_type: The type of action. It consist of a goal, a feedback and a result part.
    :param action_name: The name of the ROS2 action server that will process the goal, send feedback and return a result.
    :return: A observable message stream of the returned results.
    """
    def _send_goal(source) -> Observable:
        def _subscribe(observer, scheduler=None) -> Disposable:
            action_client = ActionClient(node, action_type, action_name)
            action_client.wait_for_server()
            source.subscribe(on_next=lambda goal: observer.on_next(action_client.send_goal(goal)))
            return observer
        return create(_subscribe)
    return _send_goal


def sample_with_frequency(frequency: float) -> Callable[[Observable], Observable]:
    """
    The operator sample_with_frequency will at regular intervals emit the last element or message
    of the observable message stream it was applied on - that is independent of whether it has
    changed or not.

    :param frequency: The frequency in HZ. A frequency of 1 (HZ) will emit a element every second.
    :return a observable data stream where the data elements are distributed with a specified frequency.
    """
    def _sample_with_frequency(source) -> Observable:
        period = 1 / frequency
        return interval(period).pipe(
            with_latest_from(source),
            map(lambda tup: tup[1]))
    return _sample_with_frequency
