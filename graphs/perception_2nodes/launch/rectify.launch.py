"""Launch a raw image publisher, pipelined with rectify and resize"""

import rclpy
import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""

    # An image_raw publisher
    image_raw_publisher = Node(
        package="perception_2nodes",
        executable="image_raw_publisher",
        name="image_raw_publisher",
        remappings=[("image_raw", "image")],
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
    return launch.LaunchDescription([image_raw_publisher, perception_container])
