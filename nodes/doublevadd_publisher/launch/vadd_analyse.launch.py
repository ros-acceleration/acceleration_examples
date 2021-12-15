#       ____  ____
#      /   /\/   /
#     /___/  \  /   Copyright (c) 2021, Xilinx®.
#     \   \   \/    Author: Víctor Mayoral Vilches <victorma@xilinx.com>
#      \   \
#      /   /
#     /___/   /\
#     \   \  /  \
#      \___\/\___\
#
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

from launch import LaunchDescription
import bt2
import sys
import datetime
import os


# Create a trace collection message iterator from the first command-line
# argument.
msg_it = bt2.TraceCollectionMessageIterator(
    str(os.environ["HOME"]) + "/.ros/tracing/vadd_capture"
)

# Last event's time (ns from origin).
last_event_ns_from_origin = None

# Iterate the trace messages and pick those ones of interest
vadd_msgs = []
for msg in msg_it:
    # `bt2._EventMessageConst` is the Python type of an event message.
    if type(msg) is bt2._EventMessageConst:

        # An event message holds a trace event.
        event = msg.event

        # Only check `sched_switch` events.
        if (
            event.name == "ros2_acceleration:vadd_pre"
            or event.name == "ros2_acceleration:vadd_post"
        ):
            vadd_msgs.append(msg)

for msg in vadd_msgs:
    if msg.event.name == "ros2_acceleration:vadd_pre":
        try:
            # Get event message's default clock snapshot's ns from origin
            # value.
            ns_from_origin_pre = msg.default_clock_snapshot.ns_from_origin
            post_msgs = [
                msg_post
                for msg_post in vadd_msgs
                if msg_post.event["iteration"] == msg.event["iteration"]
                and msg_post != msg
            ]
            if len(post_msgs) > 1:
                print("various post messages found for " + str(msg.event["iteration"]))
                print([aux_msg.event.name for aux_msg in post_msgs])
                sys.exit(1)
            else:
                post_msg = post_msgs[0]

            ns_from_origin_post = post_msg.default_clock_snapshot.ns_from_origin
            diff_ns = ns_from_origin_post - ns_from_origin_pre
            diff_ms = (ns_from_origin_post - ns_from_origin_pre) / 1e6
            diff_s = (ns_from_origin_post - ns_from_origin_pre) / 1e9

            print("vadd " + str(msg.event["iteration"]) + " → " + str(diff_ms) + " ms")

        except IndexError:
            continue


def generate_launch_description():
    return LaunchDescription()
