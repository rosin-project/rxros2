# Copyright (c) 2019, ROSIN-project
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import threading
import rclpy
from abc import abstractmethod
from rx import *
from rx.operators import *
from rx.disposable import Disposable


class Node(rclpy.node.Node):
    def __init__(self, node_name: str):
        """
        The constructor of a Node. It initialize the super class rxlcpp::Node and
        spawns a new thread that executes the 'run' method. The 'run' method is abstract
        and must therefore be implemented by a sub-class of Node.

        :param node_name: The name of the node.
        """
        super().__init__(node_name)
        threading.Thread(target=self.run).start()

    @abstractmethod
    def run(self):
        pass


def create_node(node_name: str) -> rclpy.node.Node:
    """
    create_node is a simple wrapper function that will create a new instance of a rclpy Node.

    :param node_name: The name of the node.
    :return: An instance of a newly created node.
    """
    return rclpy.create_node(node_name)


def from_topic(node: rclpy.node.Node, topic_type: Any, topic_name: str, queue_size=10) -> Observable:
    """
    The from_topic function creates an observable data stream from a ROS2 topic.

    :param node An instance of a Node.
    :param topic_type The type of the topic and also the type of the observable data elements.
    :param topic_name The name of the topic.
    :param queue_size The size of the queue associated to the topic.
    :return An observable data stream from a specified topic.
    """
    def subscribe(observer, scheduler=None) -> Disposable:
        node.create_subscription(topic_type, topic_name, lambda msg: observer.on_next(msg), queue_size)
        return observer
    return create(subscribe)


def to_topic(node: rclpy.node.Node, topic_type: Any, topic_name: str, queue_size=10) -> Callable[[Observable], Observable]:
    """
    The publish_to_topic is a special operator as it does not modify the message steam it operates on.
    However, it will take each message from the stream and publish it to a specific ROS2 topic.

    :param node An instance of a Node.
    :param topic_type The type of the observable data elements.
    :param topic_name The name of the ROS2 topic to publish the messages to.
    :param queue_size The size of the queue associated to ROS2 publisher.
    :return The observable data stream it operates on, i.e. it is an identity operator.
    """
    def _to_topic(source) -> Observable:
        publisher = node.create_publisher(topic_type, topic_name, queue_size)
        source.subscribe(on_next=lambda msg: publisher.publish(msg))
        return source
    return _to_topic


def sample_with_frequency(frequency: float) -> Callable[[Observable], Observable]:
    """
    The operator sample_with_frequency will at regular intervals emit the last element or message
    of the observable message stream it was applied on - that is independent of whether it has
    changed or not.

    :param frequency The frequency in HZ. A frequency of 1 (HZ) will emit a element every second.
    :return a observable data stream where the data elements are distributed with a specified frequency.
    """
    def _sample_with_frequency(source) -> Observable:
        period = 1 / frequency
        return interval(period).pipe(
            with_latest_from(source),
            map(lambda tup: tup[1]))
    return _sample_with_frequency
