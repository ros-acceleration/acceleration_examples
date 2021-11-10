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

An Adaptive Component built using using an adaptive_component ROS 2 package.

Publishes vadd operations while supporting different compute substrates (CPU,
FPGA), allowing for on-the-go changes following adaptive_component conventions.
*/

#include "rclcpp/rclcpp.hpp"
#include "doublevadd_component.hpp"
#include "doublevadd_component_fpga.hpp"
#include "doublevadd_component_adaptive.hpp"

using namespace std::chrono_literals;  // NOLINT
using NodeCPU = composition::DoubleVaddComponent;
using NodeFPGA = composition::DoubleVaddComponentFPGA;

namespace composition
{
DoubleVaddComponentAdaptive::DoubleVaddComponentAdaptive(const rclcpp::NodeOptions & options)
      : AdaptiveComponent("doublevadd_publisher_adaptive",        // name of the adaptive Node
      options,                                // Node options
                                              // CPU
      std::make_shared<NodeCPU>("_doublevadd_publisher_adaptive_cpu", options),
                                              // FPGA
      std::make_shared<NodeCPU>("_doublevadd_publisher_adaptive_fpga", options),
                                              // GPU
      nullptr){}

}  // namespace composition

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(composition::DoubleVaddComponentAdaptive)
