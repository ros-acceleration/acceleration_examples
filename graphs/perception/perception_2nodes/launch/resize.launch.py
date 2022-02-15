"""ResizeNode and ResizeNodeFPGA components launch file."""
import pytest
import launch
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo, Image
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


# @pytest.mark.rostest
def generate_launch_description():
    """Generate launch description with multiple components."""

    # # An image_raw publisher
    # image_raw_publisher = Node(
    #     package="perception_2nodes",
    #     executable="image_raw_publisher",
    #     name="image_raw_publisher",
    #     # remappings=[("image_raw", "image")],
    # )

    resize_container = ComposableNodeContainer(
        name="resize_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="image_proc",
                plugin="image_proc::ResizeNode",
                name="resize_node",
                # remappings=[("image", "image_raw")],  # image_raw_publisher
                remappings=[
                    ("image", "/camera/image_raw"),
                    ("camera_info", "/camera/camera_info"),
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
    return launch.LaunchDescription([resize_container])
    # return launch.LaunchDescription([image_raw_publisher, resize_container])
