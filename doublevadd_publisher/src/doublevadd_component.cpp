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

Inspired by the Vector-Add example.
See https://github.com/Xilinx/Vitis-Tutorials/blob/master/Getting_Started/Vitis
*/

#include <chrono>  // NOLINT
#include <functional>
#include <memory>
#include <string>

// #include <lttng/tracef.h>  // uncoment if using tracef
#include "tracetools/tracetools.h"
#include "tracetools_acceleration/tracetools.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "std_msgs/msg/string.hpp"
#include "doublevadd_component.hpp"
#include "vadd.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

using namespace std::chrono_literals;  // NOLINT

bool check_vadd(
          const unsigned int *in1,  // Read-Only Vector 1
          const unsigned int *in2,  // Read-Only Vector 2
          const unsigned int *out   // Read-Only Result
    ) {
  bool match = true;
  // no need to iterate twice through the loop, math's the same
  for (int i = 0 ; i < DATA_SIZE ; i++) {
      unsigned int expected = in1[i]+in2[i];
      if (out[i] != expected) {
          std::cout << "Error: Result mismatch" << std::endl;
          std::cout << "i = " << i << " CPU result = "
            << expected << " Device result = " << out[i] << std::endl;
          match = false;
          break;
      }
  }
  return match;
}

namespace composition
{

// Create a DoubleVaddNode "component" that subclasses the generic rclcpp::Node base class.
// Components get built into shared libraries and as such do not write their own main functions.
// The process using the component's shared library will instantiate the class as a ROS node.
DoubleVaddNode::DoubleVaddNode(const rclcpp::NodeOptions & options)
: Node("doublevadd_publisher", options), count_(0)
{
  // Create a publisher of "std_mgs/String" messages on the "chatter" topic.
  pub_ = create_publisher<std_msgs::msg::String>("vector", 10);

  // define differential time value (in ms) for the loop
  dt_ = 100ms;  // 10 Hz

  // Use a timer to schedule periodic message publishing.
  timer_ = create_wall_timer(dt_, std::bind(&DoubleVaddNode::on_timer, this));

}

void DoubleVaddNode::on_timer()
{
  // randomize the vectors used
  for (int i = 0 ; i < DATA_SIZE ; i++) {
      in1_[i] = rand() % DATA_SIZE;  // NOLINT
      in2_[i] = rand() % DATA_SIZE;  // NOLINT
      out_[i] = 0;
  }

  // Add vectors
  // tracef("ros2:krs:doublevadd_publisher:main calling vadd");
  TRACEPOINT(vadd_pre, ("iteration: " + std::to_string(publish_count_)).c_str());
  vadd(in1_, in2_, out_, DATA_SIZE);  // function subject to be accelerated
  TRACEPOINT(vadd_post, ("iteration: " + std::to_string(publish_count_)).c_str());

  // Validate operation
  check_vadd(in1_, in2_, out_);

  // Publish publish result
  std_msgs::msg::String message;
  message.data = "vadd finished, iteration: " +
    std::to_string(publish_count_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

  try {
    // Put the message into a queue to be processed by the middleware.
    // This call is non-blocking.
    pub_->publish(std::move(message));
  } catch (const rclcpp::exceptions::RCLError & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "unexpectedly failed with %s",
      e.what());
  }
} // on_timer

}  // namespace composition

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(composition::DoubleVaddNode)
