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

"""Launch power in a component container."""

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from tracetools_launch.action import Trace
from tracetools_trace.tools.names import DEFAULT_CONTEXT
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS


def generate_launch_description():
    """Generate launch description with multiple components."""

    trace = Trace(
        session_name="power_capture",
        events_ust=[
            "ros2_acceleration:*",
        ]
        + DEFAULT_EVENTS_ROS,
        # events_kernel=[
        #     "sched_switch",
        # ],
    )

    container = ComposableNodeContainer(
        name="my_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="ros2_kria_power", plugin="composition::Power", name="power"
            ),
            ComposableNode(
                package="doublevadd_publisher",
                plugin="composition::DoubleVaddNode",
                name="doublevadd_publisher",
            ),
        ],
        output="screen",
    )

    return launch.LaunchDescription([container, trace])
