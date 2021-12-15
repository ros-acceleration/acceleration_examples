"""RectifyNode and RectifyNodeFPGA components launch file."""

import os
import pathlib
import time
import cv2

# from isaac_ros_test import IsaacROSBaseTest, JSONConversion
# import launch_ros
import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo, Image
from launch_ros.actions import Node

# from sklearn.linear_model import LinearRegression

import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


# @pytest.mark.rostest
def generate_launch_description():
    """Generate launch description with multiple components."""

    # An image_raw publisher
    image_raw_publisher = Node(
        package="perception_2nodes",
        executable="image_raw_publisher",
        name="image_raw_publisher",
        # remappings=[("image_raw", "image")],
    )

    rectify_container = ComposableNodeContainer(
        name="rectify_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="image_proc",
                plugin="image_proc::RectifyNodeFPGA",
                name="rectify_node_fpga",
                remappings=[("image", "image_raw")],
            ),
            # ComposableNode(
            #     package="image_proc",
            #     plugin="image_proc::RectifyNode",
            #     name="rectify_node",
            #     remappings=[("image", "image_raw")],
            # ),
        ],
        output="screen",
    )

    # return launch.LaunchDescription([image_raw_publisher])
    # return launch.LaunchDescription([rectify_container])
    return launch.LaunchDescription([image_raw_publisher, rectify_container])
