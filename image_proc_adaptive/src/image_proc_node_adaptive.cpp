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


An image_proc adaptive ROS 2 Node using Components.

Supports CPU or FPGA computations, allows for on-the-go changes following
adaptive_component conventions (see https://github.com/ros-acceleration/adaptive_component).
*/

#include "rclcpp/rclcpp.hpp"
#include "image_proc_adaptive.hpp"

int main(int argc, char * argv[])
{
  using AdaptiveNode = composition::ResizeNodeAdaptive;

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::executors::MultiThreadedExecutor exec;

  // Make an Adaptive Node out of an Adaptive Component
  auto adaptive_node_with_components_and_state = std::make_shared<AdaptiveNode>(options);
  exec.add_node(adaptive_node_with_components_and_state);

  exec.spin();  // spin the executor
  rclcpp::shutdown();
  return 0;
}
