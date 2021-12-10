#    ____  ____
#   /   /\/   /
#  /___/  \  /   Copyright (c) 2021, Xilinx®.
#  \   \   \/    Author: Víctor Mayoral Vilches <victorma@xilinx.com>
#   \   \
#   /   /
#  /___/   /\
#  \   \  /  \
#   \___\/\___\
#
# Licensed under the Apache License, Version 2.0
#


from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node


def generate_launch_description():

    container = ComposableNodeContainer(
        name="resize_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            # TODO: consider subclassing ComposableNode and creating an
            # AdaptiveNode class that allows for launch-time remapping of
            # names, topics, etc.
            ComposableNode(
                package="image_proc_adaptive",
                plugin="composition::ResizeNodeAdaptive",
                name="ResizeNode_adaptive",
            ),
            # ComposableNode(
            #     package="image_proc",
            #     plugin="image_proc::ResizeNode",
            #     name="resize_node",
            # ),
            # ComposableNode(
            #     package="image_proc",
            #     plugin="image_proc::ResizeNodeFPGA",
            #     name="resize_fpga_node",
            # ),
        ],
        output="screen",
    )

    return LaunchDescription([container])
