"""ResizeNode and ResizeNodeFPGA components launch file."""

import launch
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from tracetools_launch.action import Trace
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS
from tracetools_trace.tools.names import DEFAULT_EVENTS_KERNEL
from tracetools_trace.tools.names import DEFAULT_CONTEXT


def generate_launch_description():
    """Generate launch description with multiple components."""
     
     # Trace
    trace = Trace(
        session_name="trace_resize",
        events_ust=[
            "ros2_image_pipeline:*",
            "lttng_ust_cyg_profile*",
            "lttng_ust_statedump*",
            "liblttng-ust-libc-wrapper",
        ]
        + DEFAULT_EVENTS_ROS,
        context_fields={
                'kernel': [],
                'userspace': ['vpid', 'vtid', 'procname'],
        },
        events_kernel=DEFAULT_EVENTS_KERNEL,
        # context_names=DEFAULT_CONTEXT,
    )

    rectify_container = ComposableNodeContainer(
        name="rectify_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            # ComposableNode(
            #     package="image_proc",
            #     plugin="image_proc::ResizeNodeFPGA",
            #     name="rectify_node_fpga",
            #     remappings=[("image", "image_raw")],
            # ),
            ComposableNode(
                package="image_proc",
                plugin="image_proc::ResizeNode",
                name="rectify_node",
                # remappings=[("image", "image_raw")],  # for image_raw_publisher
                remappings=[
                    ("image", "/camera/image_raw"),
                    ("camera_info", "/camera/camera_info"),
                ],
            ),
        ],
        output="screen",
    )

    return launch.LaunchDescription([rectify_container, trace])