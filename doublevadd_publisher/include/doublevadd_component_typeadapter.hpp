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

#ifndef DOUBLEVADD_COMPONENT_TYPEADAPTER_HPP_
#define DOUBLEVADD_COMPONENT_TYPEADAPTER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>

#define DATA_SIZE 4096  // 2**12
// #define DATA_SIZE 16384  // 2**14
// #define DATA_SIZE 65536  // 2**16
// #define DATA_SIZE 262144  // 2**18

template<>
struct rclcpp::TypeAdapter<std::string, std_msgs::msg::String>
{
  using is_specialized = std::true_type;
  using custom_type = std::string;
  using ros_message_type = std_msgs::msg::String;

  static
  void
  convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination)
  {
    destination.data = source;
  }

  static
  void
  convert_to_custom(
    const ros_message_type & source,
    custom_type & destination)
  {
    destination = source.data;
  }
};  // rclcpp::TypeAdapter

namespace composition
{

class DoubleVaddTypeAdapter : public rclcpp::Node
{
  using MyAdaptedType = rclcpp::TypeAdapter<std::string, std_msgs::msg::String>;

public:
  explicit DoubleVaddTypeAdapter(const rclcpp::NodeOptions & options);

protected:
  void on_timer();

private:
  size_t count_;
  rclcpp::Publisher<MyAdaptedType>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::milliseconds dt_;

  unsigned int in1_[DATA_SIZE];
  unsigned int in2_[DATA_SIZE];
  unsigned int out_[DATA_SIZE];

  int publish_count_ = 0;

};

}  // namespace "composition"

#endif  // DOUBLEVADD_COMPONENT_TYPEADAPTER_HPP_
