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

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from tracetools_launch.action import Trace
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS
from tracetools_trace.tools.names import DEFAULT_EVENTS_KERNEL
from tracetools_trace.tools.names import DEFAULT_CONTEXT
 
def generate_launch_description():
     # Trace
    trace = Trace(
        session_name="trace_rectify",
        events_ust=[
            "ros2_image_pipeline:*",
            "lttng_ust_cyg_profile*",
            "lttng_ust_statedump*",
        ]
        + DEFAULT_EVENTS_ROS, 
        context_fields={
                'kernel': [],
                'userspace': ['vpid', 'vtid', 'procname'],
        },
        events_kernel=DEFAULT_EVENTS_KERNEL,
        # context_names=DEFAULT_CONTEXT,
    )

    # An image_raw publisher
    publisher_xilinx = Node(
        package="image_pipeline_examples",
        executable="rectify_node",
        name="rectify_node",
        remappings=[
                    ("image", "/camera/image_raw"),
                    ("camera_info", "/camera/camera_info"),
        ],
    )

    return LaunchDescription([
        trace,
        publisher_xilinx
    ])
