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

#ifndef DOUBLEVADD_COMPONENT_HPP_
#define DOUBLEVADD_COMPONENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>

#define DATA_SIZE 4096  // 2**12
// #define DATA_SIZE 16384  // 2**14
// #define DATA_SIZE 65536  // 2**16
// #define DATA_SIZE 262144  // 2**18

namespace composition
{

class DoubleVaddComponent : public rclcpp::Node
{
public:
  explicit DoubleVaddComponent(const rclcpp::NodeOptions & options)
      : Node("doublevadd_publisher", options), count_(0) {initialize();}

  explicit DoubleVaddComponent(const std::string &node_name,
                               const rclcpp::NodeOptions & options)
      : Node(node_name, options), count_(0) {initialize();}

  int publish_count_ = 0;

protected:
  void on_timer();

private:
  void initialize();

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::milliseconds dt_;

  unsigned int in1_[DATA_SIZE];
  unsigned int in2_[DATA_SIZE];
  unsigned int out_[DATA_SIZE];

  size_t count_;

};

}  // namespace composition

#endif  // DOUBLEVADD_COMPONENT_HPP_
