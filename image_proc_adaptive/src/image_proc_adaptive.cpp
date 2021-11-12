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


An image_proc adaptive ROS 2 Component.

Supports CPU or FPGA computations, allows for on-the-go changes following
adaptive_component conventions (see https://github.com/ros-acceleration/adaptive_component).
*/

#include "rclcpp/rclcpp.hpp"
#include "image_proc/resize.hpp"
#include "image_proc/resize_fpga.hpp"
#include "image_proc_adaptive.hpp"

using namespace std::chrono_literals;  // NOLINT
using NodeCPU = image_proc::ResizeNode;
using NodeFPGA = image_proc::ResizeNodeFPGA;

namespace composition
{
ResizeNodeAdaptive::ResizeNodeAdaptive(const rclcpp::NodeOptions & options)
      : AdaptiveComponent("ResizeNode_adaptive",        // name of the adaptive Node
      options,                                // Node options
                                              // CPU
      std::make_shared<NodeCPU>(options),
                                              // FPGA
      std::make_shared<NodeFPGA>(options),
                                              // GPU
      nullptr){}

}  // namespace composition

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(composition::ResizeNodeAdaptive)
