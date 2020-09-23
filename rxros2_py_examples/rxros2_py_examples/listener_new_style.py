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
