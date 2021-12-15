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

"""Example launch file for a profiling analysis."""

from launch import LaunchDescription
from launch_ros.actions import Node
from tracetools_launch.action import Trace
from tracetools_trace.tools.names import DEFAULT_CONTEXT
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS


def generate_launch_description():
    return LaunchDescription(
        [
            Trace(
                session_name="faster_doublevadd_publisher",
                events_ust=[
                    "ros2_acceleration:vadd_pre",
                    "ros2_acceleration:vadd_post",
                ]
                + DEFAULT_EVENTS_ROS,
                events_kernel=[
                    "sched_switch",
                ],
                # context_names=[
                #     "ip",
                # ]
                # + DEFAULT_CONTEXT,
            ),
            Node(
                package="faster_doublevadd_publisher",
                executable="faster_doublevadd_publisher",
                # arguments=["do_more"],
                output="screen",
            ),
        ]
    )
