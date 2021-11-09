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
#include "std_msgs/msg/string.hpp"
#include "vadd.hpp"

#include "doublevadd_component.hpp"
#include "doublevadd_component_fpga.hpp"
#include "adaptive_component/adaptive_component.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::executors::MultiThreadedExecutor exec;
  using NodeCPU = composition::DoubleVaddComponent;
  using NodeFPGA = composition::DoubleVaddComponentFPGA;

  // Create an adaptive ROS 2 Node using "components", the resulting
  // Node is also programed as a "component", retaining composability
  auto adaptive_node = std::make_shared<composition::AdaptiveComponent>(
        "doublevadd_publisher_adaptive",        // name of the adaptive Node
        options,                                // Node options
                                                // CPU
        std::make_shared<NodeCPU>("_doublevadd_publisher_adaptive_cpu", options),
                                                // FPGA
        std::make_shared<NodeFPGA>("_doublevadd_publisher_adaptive_fpga", options),
                                                // GPU
        nullptr);

  // fill up the executor
  exec.add_node(adaptive_node);

  // // Another Node
  // auto doublevadd_node3 = std::make_shared<composition::DoubleVaddComponent>("doublevadd_node3", options);
  // exec.add_node(doublevadd_node3);

  exec.spin();  // spin the executor
  rclcpp::shutdown();
  return 0;
}
