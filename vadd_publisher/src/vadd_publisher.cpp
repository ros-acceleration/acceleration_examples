// Copyright (c) 2021, Xilinx®.
// All rights reserved
//
// Inspired by the Vector-Add example.
// See https://github.com/Xilinx/Vitis-Tutorials/blob/master/Getting_Started/Vitis
//
// Author: Víctor Mayoral Vilches <v.mayoralv@gmail.com>

#include <chrono>  // NOLINT
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "vadd.hpp"

// #define DATA_SIZE 4096  // 2**12
#define DATA_SIZE 16384  // 2**14
// #define DATA_SIZE 65536  // 2**16
// #define DATA_SIZE 262144  // 2**18

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

int main(int argc, char * argv[]) {
  // ROS 2 abstractions
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("vadd_publisher");
  auto publisher = node->create_publisher<std_msgs::msg::String>("vector", 10);
  auto publish_count = 0;
  std_msgs::msg::String message;
  rclcpp::WallRate loop_rate(100ms);

  // Application variables
  unsigned int in1[DATA_SIZE];
  unsigned int in2[DATA_SIZE];
  unsigned int out[DATA_SIZE];

  while (rclcpp::ok()) {
    // randomize the vectors used
    for (int i = 0 ; i < DATA_SIZE ; i++) {
        in1[i] = rand() % DATA_SIZE;  // NOLINT
        in2[i] = rand() % DATA_SIZE;  // NOLINT
        out[i] = 0;
    }

    // Add vectors
    vadd(in1, in2, out, DATA_SIZE);  // function subject to be accelerated

    // Validate operation
    check_vadd(in1, in2, out);

    // Publish publish result
    message.data = "vadd finished, iteration: " +
      std::to_string(publish_count++);
    RCLCPP_INFO(node->get_logger(), "Publishing: '%s'", message.data.c_str());

    try {
      publisher->publish(message);
      rclcpp::spin_some(node);
    } catch (const rclcpp::exceptions::RCLError & e) {
      RCLCPP_ERROR(
        node->get_logger(),
        "unexpectedly failed with %s",
        e.what());
    }
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
