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

#ifndef DOUBLEVADD_COMPONENT_FPGA_HPP_
#define DOUBLEVADD_COMPONENT_FPGA_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <vitis_common/common/ros_opencl_120.hpp>
#include <chrono>

#define DATA_SIZE 4096  // 2**12
// #define DATA_SIZE 16384  // 2**14
// #define DATA_SIZE 65536  // 2**16
// #define DATA_SIZE 262144  // 2**18

namespace composition
{

class DoubleVaddComponentFPGA : public rclcpp::Node
{
public:
  explicit DoubleVaddComponentFPGA(const rclcpp::NodeOptions & options)
      : Node("doublevadd_publisher", options), count_(0) {initialize(options);}

  explicit DoubleVaddComponentFPGA(const std::string &node_name,
                                   const rclcpp::NodeOptions & options)
      : Node(node_name, options), count_(0) {initialize(options);}

  ~DoubleVaddComponentFPGA()
  {
      // Free memory for OpenCL buffers
      delete in1_buf_;
      delete in2_buf_;
      delete out_buf_;

      // Free memory for other OpenCL constructs
      delete q_;
      delete krnl_vector_add_;

  } // DoubleVaddComponentFPGA destructor

  int publish_count_ = 0;

protected:
  void on_timer();
  void init_fpga();
  bool check_compute(
            const unsigned int *in1,  // Read-Only Vector 1
            const unsigned int *in2,  // Read-Only Vector 2
            const unsigned int *out,  // Read-Only Result
            int size);                // Size in integer
  void compute_fpga(
          const unsigned int *in1,  // Read-Only Vector 1
          const unsigned int *in2,  // Read-Only Vector 2
          unsigned int *out,        // Output Result
          int size);                // Size in integer


private:
  void initialize(const rclcpp::NodeOptions & options);

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::milliseconds dt_;

  // Buffers for manipulating memory mapped to the FPGA
  // (the OpenCL buffers)
  unsigned int *in1_fpga_;
  unsigned int *in2_fpga_;
  unsigned int *out_fpga_;

  // OpenCL buffers
  cl::Buffer *in1_buf_;
  cl::Buffer *in2_buf_;
  cl::Buffer *out_buf_;

  // OpenCL abstractions for kernel management
  cl::CommandQueue *q_;
  cl::Kernel *krnl_vector_add_;

  size_t count_;
  bool fpga_initialized_ = false;

  int argc_;
  char **argv_;
};

}  // namespace composition

#endif  // DOUBLEVADD_COMPONENT_FPGA_HPP_
