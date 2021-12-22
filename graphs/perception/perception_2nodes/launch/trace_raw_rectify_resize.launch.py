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

"""Launch a raw image publisher, pipelined with rectify and resize"""

import os
import rclpy
import launch
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo, Image
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from tracetools_launch.action import Trace
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS


def generate_launch_description():
    """Generate launch description with multiple components."""

    # An image_raw publisher
    image_raw_publisher = Node(
        package="perception_2nodes",
        executable="image_raw_publisher",
        name="image_raw_publisher",
        remappings=[("image_raw", "image")],
    )

    # Trace
    trace = Trace(
        session_name="raw_rectify_resize_pipeline",
        events_ust=[
            "ros2_image_pipeline:*",
        ]
        + DEFAULT_EVENTS_ROS,
    )

    perception_container = ComposableNodeContainer(
        name="perception_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            # ComposableNode(
            #     package="image_proc",
            #     plugin="image_proc::RectifyNodeFPGA",
            #     name="rectify_node_fpga",
            #     remappings=[("image", "image_raw")],
            # ),
            ComposableNode(
                package="image_proc",
                plugin="image_proc::RectifyNode",
                name="rectify_node",
            ),
            ComposableNode(
                namespace="resize",
                package="image_proc",
                plugin="image_proc::ResizeNode",
                name="resize_node",
                remappings=[
                    ("camera_info", "/camera_info"),
                    ("image", "/image_rect"),
                    ("resize", "resize"),
                ],
                parameters=[
                    {
                        "scale_height": 2.0,
                        "scale_width": 2.0,
                    }
                ],
            ),
        ],
        output="screen",
    )
    return launch.LaunchDescription([image_raw_publisher, perception_container, trace])
