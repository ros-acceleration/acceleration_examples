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

"""Launch various doublevadd publishers, each using a different piece of the FPGA (if any)"""

import os
import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
# from tracetools_launch.action import Trace
# from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS

def generate_launch_description():
    """Generate launch description with multiple components."""

    # Accelerator argument
    accelerator_argument = DeclareLaunchArgument(
        name='accelerator',
        default_value='/lib/firmware/xilinx/multiple_doublevadd_publisher/multiple.xclbin',
        description='Absolute path to the acceleration kernel'
    )

    # offloaded
    offloaded_doublevadd_publisher = Node(
        package="multiple_doublevadd_publisher",
        executable="offloaded_doublevadd_publisher",
        name="offloaded_doublevadd_publisher",
        remappings=[("vector_acceleration", "vector_acceleration_offloaded")],
        output='screen',
        arguments = [LaunchConfiguration('accelerator')]
    )

    # accelerated
    accelerated_doublevadd_publisher = Node(
        package="multiple_doublevadd_publisher",
        executable="accelerated_doublevadd_publisher",
        name="accelerated_doublevadd_publisher",
        remappings=[("vector_acceleration", "vector_acceleration_accelerated")],
        output='screen',
        arguments = [LaunchConfiguration('accelerator')]
    )

    # faster
    faster_doublevadd_publisher = Node(
        package="multiple_doublevadd_publisher",
        executable="faster_doublevadd_publisher",
        name="faster_doublevadd_publisher",
        remappings=[("vector_acceleration", "vector_acceleration_faster")],
        output='screen',
        arguments = [LaunchConfiguration('accelerator')]
    )

    return launch.LaunchDescription(
        [
            accelerator_argument,
            offloaded_doublevadd_publisher, 
            accelerated_doublevadd_publisher, 
            faster_doublevadd_publisher
        ]
    )
