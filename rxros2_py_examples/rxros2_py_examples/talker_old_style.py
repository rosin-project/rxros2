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


def mk_msg(s) -> String:
    msg = String()
    msg.data = s
    return msg


def main(args=None):
    rclpy.init(args=args)
    talker = rxros2.create_node("talker")

    rxros2.interval(1.0).pipe(
        rxros2.map(lambda i: mk_msg("Hello world " + str(i))),
        rxros2.do_action(lambda s: print("Send2 {0}".format(s.data))),
        rxros2.sample_with_frequency(2.0),
        rxros2.publish_to_topic(talker, String, "/chatter"))

    rclpy.spin(talker)
    talker.destroy_node()
    rclpy.shutdown()
