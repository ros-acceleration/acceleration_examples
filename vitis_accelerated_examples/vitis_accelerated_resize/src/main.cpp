/*
      ____  ____
     /   /\/   /
    /___/  \  /   Copyright (c) 2024, AMD®.
    \   \   \/    Author: Jasvinder Khurana <jasvinder.khurana@amd.com>
     \   \
     /   /        Licensed under the Apache License, Version 2.0 (the "License");
    /___/   /\    you may not use this file except in compliance with the License.
    \   \  /  \   You may obtain a copy of the License at
     \___\/\___\            http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.

*/

#include <chrono>  // NOLINT
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "resize_fpga.hpp"
#include "minimalimagepublisher.hpp"
#include <vitis_common/common/ros_opencl_120.hpp>
#include <vitis_common/common/utilities.hpp>

using namespace std::chrono_literals;  // NOLINT

int main(int argc, char * argv[]) {
  // ROS 2 abstractions
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;

  rclcpp::NodeOptions node_options;
  auto node_accelerated_resize		= std::make_shared<AcceleratedResize>(node_options);
  auto node_minimal_publisher 		= std::make_shared<MinimalImagePublisher>();


  executor.add_node(node_minimal_publisher);
  executor.add_node(node_accelerated_resize);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
