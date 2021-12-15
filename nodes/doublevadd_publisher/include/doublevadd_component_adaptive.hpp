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

#ifndef DOUBLEVADD_COMPONENT_ADAPTIVE_HPP_
#define DOUBLEVADD_COMPONENT_ADAPTIVE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "doublevadd_component.hpp"
#include "doublevadd_component_fpga.hpp"
#include "adaptive_component/adaptive_component.hpp"

namespace composition
{

/// An Adaptive Component built using using an adaptive_component ROS 2 package.
/*
 * Publishes vadd operations while supporting different compute substrates (CPU,
 * FPGA), allowing for on-the-go changes following adaptive_component conventions.
 */
class DoubleVaddComponentAdaptive : public AdaptiveComponent
{
public:
  /// Default constructor
  explicit DoubleVaddComponentAdaptive(const rclcpp::NodeOptions & options);
};

}  // namespace composition

#endif  // DOUBLEVADD_COMPONENT_ADAPTIVE_HPP_
