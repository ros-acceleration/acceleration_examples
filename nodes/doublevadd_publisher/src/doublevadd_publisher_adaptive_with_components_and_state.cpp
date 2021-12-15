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

An Adaptive Node built out of "Components". The Node defaults to
CPU computations and can dynamically offload to the FPGA and adapt
accordingly.

The Node leverages the "AdaptiveComponent" class to simplify the creation of
adaptive Node behaviors through parameters.
*/

#include "rclcpp/rclcpp.hpp"
#include "doublevadd_component_adaptive_stateful.hpp"

int main(int argc, char * argv[])
{
  using AdaptiveNode = composition::DoubleVaddComponentAdaptiveStateful;

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
