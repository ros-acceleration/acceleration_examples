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
from wasabi import color

# color("{:02x}".format(x), fg=16, bg="green")
# debug = True  # debug flag, set to True if desired

def get_change(first, second):
    """
    Get change in percentage between two values
    """
    if first == second:
        return 0
    try:
        return (abs(first - second) / second) * 100.0
    except ZeroDivisionError:
        return float("inf")


# Create a trace collection message iterator from the first command-line
# argument.
msg_it = bt2.TraceCollectionMessageIterator(
    str(os.environ["HOME"]) + "/.ros/tracing/raw_rectify_resize_pipeline"
)

# Iterate the trace messages and pick those ones of interest
image_pipeline_msgs = []
for msg in msg_it:
    # `bt2._EventMessageConst` is the Python type of an event message.
    if type(msg) is bt2._EventMessageConst:

        # An event message holds a trace event.
        event = msg.event

        # Only check `sched_switch` events.
        if (
            event.name == "ros2_image_pipeline:image_proc_rectify_init"
            or event.name == "ros2_image_pipeline:image_proc_rectify_fini"
            or event.name == "ros2_image_pipeline:image_proc_resize_init"
            or event.name == "ros2_image_pipeline:image_proc_resize_fini"
        ):
            image_pipeline_msgs.append(msg)

# Form sets with each pipeline
image_pipeline_msg_sets = []
new_set = []
for msg in image_pipeline_msgs:
    if msg.event.name == "ros2_image_pipeline:image_proc_resize_fini":
        new_set.append(msg)
        image_pipeline_msg_sets.append(new_set)
        new_set = []
    else:
        new_set.append(msg)

for msg_set in image_pipeline_msg_sets:
    if len(msg_set) != 4:
        print(color("Not a complete set: " + str(msg_set), fg="red"))
    else:
        rectify_init_msg = msg_set[0]
        rectify_fini_msg = msg_set[1]
        resize_init_msg = msg_set[2]
        resize_fini_msg = msg_set[3]

        ns_from_origin_rectify_init = rectify_init_msg.default_clock_snapshot.ns_from_origin
        ns_from_origin_rectify_fini = rectify_fini_msg.default_clock_snapshot.ns_from_origin
        ns_from_origin_resize_init = resize_init_msg.default_clock_snapshot.ns_from_origin
        ns_from_origin_resize_fini = resize_fini_msg.default_clock_snapshot.ns_from_origin

        # rectify
        diff_rectify_ns = ns_from_origin_rectify_fini - ns_from_origin_rectify_init
        diff_rectify_ms = diff_rectify_ns / 1e6
        diff_rectify_s = diff_rectify_ns / 1e9

        # resize
        diff_resize_ns = ns_from_origin_resize_fini - ns_from_origin_resize_init
        diff_resize_ms = diff_resize_ns / 1e6
        diff_resize_s = diff_resize_ns / 1e9

        # all, rectify + resize
        diff_all_ns = ns_from_origin_resize_fini - ns_from_origin_rectify_init
        diff_all_ms = diff_all_ns / 1e6
        diff_all_s = diff_all_ns / 1e9


        stringout = color("raw image → ") + \
                    color("rectify ({} ms) → ".format(diff_rectify_ms), fg="black", bg="grey") + \
                    color("resize ({} ms) → ".format(diff_resize_ms), fg="white", bg="black") + \
                    color("all ({} ms)".format(diff_all_ms), fg="black", bg="red")

        print(stringout)
        # print("vadd " + str(msg.event["iteration"]) + " → " + str(diff_ms) + " ms")

def generate_launch_description():
    return LaunchDescription()
