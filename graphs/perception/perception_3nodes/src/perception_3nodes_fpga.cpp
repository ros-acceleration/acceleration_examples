// Copyright 2022 Víctor Mayoral-Vilches
// All rights reserved.

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "harris_fpga.hpp"
#include "rectify_fpga.hpp"
#include "resize_fpga.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::executors::MultiThreadedExecutor exec;

  rclcpp::NodeOptions options;
  auto rectify_node =
    std::make_shared<perception_3nodes::RectifyNodeFPGAStreamlined>(options);
  auto resize_node =
    std::make_shared<perception_3nodes::ResizeNodeFPGAStreamlined>(options);
  auto harris_node =
    std::make_shared<perception_3nodes::HarrisNodeFPGA>(options);

  exec.add_node(rectify_node);
  exec.add_node(resize_node);
  exec.add_node(harris_node);

  exec.spin();
  rclcpp::shutdown();
  return 0;
}
