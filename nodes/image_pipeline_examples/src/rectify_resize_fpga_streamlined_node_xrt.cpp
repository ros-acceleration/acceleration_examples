/*
      ____  ____
     /   /\/   /
    /___/  \  /   Copyright (c) 2021, Xilinx®.
    \   \   \/    Author: Víctor Mayoral Vilches <victorma@xilinx.com>
     \   \
     /   /
    /___/   /\
    \   \  /  \
     \___\/\___\

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

// An image_pipeline rectify component into a node for tracing purposes.
//  see https://gitlab.com/ros-tracing/ros2_tracing/-/issues/137

#include <memory>
#include "image_proc/rectify_fpga_streamlined_xrt.hpp"
#include "image_proc/resize_fpga_streamlined_xrt.hpp"
// #include "image_proc/resize_fpga_streamlined.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::executors::MultiThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto resize_node =
    std::make_shared<image_proc::ResizeNodeFPGAStreamlinedXRT>(options);
  auto rectify_node =
    std::make_shared<image_proc::RectifyNodeFPGAStreamlinedXRT>(options);

  exec.add_node(rectify_node);
  exec.add_node(resize_node);

  exec.spin();
  rclcpp::shutdown();
  return 0;
}
