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

A trivial double vector-add ROS 2 publisher built using "components",
that runs in the FPGA, adaptively.

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
#include "vadd.hpp"
#include "doublevadd_component_fpga.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include <vitis_common/common/ros_opencl_120.hpp>
#include <vitis_common/common/utilities.hpp>

#define DATA_SIZE 4096  // 2**12
// #define DATA_SIZE 16384  // 2**14
// #define DATA_SIZE 65536  // 2**16
// #define DATA_SIZE 262144  // 2**18


// rcl_context_impl_s mapped to rcl_context_impl_t and defined
// for internal uses at https://github.com/ros2/rcl/blob/master/rcl/src/rcl/context_impl.h#L29
struct rcl_context_impl_t
{
  /// Allocator used during init and shutdown.
  rcl_allocator_t allocator;
  /// Copy of init options given during init.
  rcl_init_options_t init_options;
  /// Length of argv (may be `0`).
  int64_t argc;
  /// Copy of argv used during init (may be `NULL`).
  char ** argv;
  /// rmw context.
  rmw_context_t rmw_context;
};

using namespace std::chrono_literals;  // NOLINT

namespace composition
{

void
DoubleVaddComponentFPGA::initialize(const rclcpp::NodeOptions & options)
{
  // Create a publisher of "std_mgs/String" messages on the "chatter" topic.
  pub_ = create_publisher<std_msgs::msg::String>("vector", 10);

  // define differential time value (in ms) for the loop
  dt_ = 100ms;  // 10 Hz

  // Use a timer to schedule periodic message publishing.
  timer_ = create_wall_timer(dt_, std::bind(&DoubleVaddComponentFPGA::on_timer, this));

  // Fetch arguments from rcl
  auto context = options.context();
  auto rcl_context = context->get_rcl_context();
  argc_ = rcl_context->impl->argc;
  argv_ = rcl_context->impl->argv;

  // Initialize the FPGA
  init_fpga();
}

/// `init_fpga`
/**
 * Initialize memory and OpenCL abstractions for interacting with the FPGA
 */
void
DoubleVaddComponentFPGA::init_fpga()
{
  // ------------------------------------------------------------------------
  // Step 1: Initialize the OpenCL environment for acceleration
  // ------------------------------------------------------------------------
  cl_int err;
  std::string binaryFile = (argc_ < 2) ? "vadd.xclbin" : argv_[1];

  // // Debug values
  // //
  // RCLCPP_INFO(this->get_logger(), "argc: '%d'", argc_);
  // RCLCPP_INFO(this->get_logger(), "argv: '%s'", argv_[1]);

  unsigned fileBufSize;
  std::vector<cl::Device> devices = get_xilinx_devices();
  devices.resize(1);
  cl::Device device = devices[0];
  cl::Context context(device, NULL, NULL, NULL, &err);
  char* fileBuf = read_binary_file(binaryFile, fileBufSize);
  cl::Program::Binaries bins{{fileBuf, fileBufSize}};
  cl::Program program(context, devices, bins, NULL, &err);
  q_ = new cl::CommandQueue(context, device, CL_QUEUE_PROFILING_ENABLE, &err);
  krnl_vector_add_ = new cl::Kernel(program, "vadd", &err);

  // ------------------------------------------------------------------------
  // Step 2: Create buffers, map memory
  // ------------------------------------------------------------------------
  // Create the buffers and allocate memory
  in1_buf_ = new cl::Buffer(context, CL_MEM_ALLOC_HOST_PTR | CL_MEM_READ_ONLY,
    sizeof(unsigned int) * DATA_SIZE, NULL, &err);
  in2_buf_ = new cl::Buffer(context, CL_MEM_ALLOC_HOST_PTR | CL_MEM_READ_ONLY,
    sizeof(unsigned int) * DATA_SIZE, NULL, &err);
  out_buf_ = new cl::Buffer(context, CL_MEM_ALLOC_HOST_PTR | CL_MEM_WRITE_ONLY,
    sizeof(unsigned int) * DATA_SIZE, NULL, &err);

  // Map buffers to kernel arguments, thereby assigning
  //  them to specific device memory banks
  krnl_vector_add_->setArg(0, *in1_buf_);
  krnl_vector_add_->setArg(1, *in2_buf_);
  krnl_vector_add_->setArg(2, *out_buf_);

  // Map host-side buffer memory to user-space pointers
  in1_fpga_ = (unsigned int *)q_->enqueueMapBuffer(*in1_buf_, CL_TRUE, CL_MAP_WRITE, 0, sizeof(unsigned int) * DATA_SIZE);  // NOLINT
  in2_fpga_ = (unsigned int *)q_->enqueueMapBuffer(*in2_buf_, CL_TRUE, CL_MAP_WRITE, 0, sizeof(unsigned int) * DATA_SIZE);  // NOLINT
  out_fpga_ = (unsigned int *)q_->enqueueMapBuffer(*out_buf_, CL_TRUE, CL_MAP_WRITE | CL_MAP_READ, 0, sizeof(unsigned int) * DATA_SIZE);  // NOLINT

  // Mark as initialized
  fpga_initialized_ = true;
}


/// `compute_fpga`
/**
 * Perform vector-add Node computations within the CPU.
 *
 * \param[in] in1, vector array operand 1
 * \param[in] in2, vector array operand 2
 * \param[out] out, vector array with the resulting addition
 * \param[in] size, size of the vectors
 */
void
DoubleVaddComponentFPGA::compute_fpga(
        const unsigned int *in1,  // Read-Only Vector 1
        const unsigned int *in2,  // Read-Only Vector 2
        unsigned int *out,        // Output Result
        int size)                 // Size in integer
{
  TRACEPOINT(vadd_pre, ("iteration: " + std::to_string(publish_count_)).c_str());
  // Set kernel arguments
  krnl_vector_add_->setArg(0, *in1_buf_);
  krnl_vector_add_->setArg(1, *in2_buf_);
  krnl_vector_add_->setArg(2, *out_buf_);
  krnl_vector_add_->setArg(3, size);

  // Schedule transfer of inputs to device memory
  q_->enqueueMigrateMemObjects({*in1_buf_, *in2_buf_}, 0 /* 0 means from host*/);
  // execution of kernel
  q_->enqueueTask(*krnl_vector_add_);
  // transfer of outputs back to host memory
  q_->enqueueMigrateMemObjects({*out_buf_}, CL_MIGRATE_MEM_OBJECT_HOST);
  // Wait for all scheduled operations to finish
  q_->finish();
  TRACEPOINT(vadd_post, ("iteration: " + std::to_string(publish_count_)).c_str());

  // Validate operation
  check_compute(in1, in2, out, size);
}

/// `check_compute`
/**
 * Verify that computation's result is valid.
 *
 * \param[in] in1, vector array operand 1
 * \param[in] in2, vector array operand 2
 * \param[in] out, result of the computation
 */
bool
DoubleVaddComponentFPGA::check_compute(
          const unsigned int *in1,  // Read-Only Vector 1
          const unsigned int *in2,  // Read-Only Vector 2
          const unsigned int *out,  // Read-Only Result
          int size)                 // Size in integer
{
  bool match = true;
  // no need to iterate twice through the loop, math's the same
  for (int i = 0 ; i < size ; i++) {
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

void
DoubleVaddComponentFPGA::on_timer()
{
  // Re-initialize FPGA if appropriate
  if (!fpga_initialized_) {
    init_fpga();
  }

  // randomize the vectors used
  for (int i = 0 ; i < DATA_SIZE ; i++) {
      in1_fpga_[i] = rand() % DATA_SIZE;  // NOLINT
      in2_fpga_[i] = rand() % DATA_SIZE;  // NOLINT
      out_fpga_[i] = 0;
  }

  compute_fpga(in1_fpga_, in2_fpga_, out_fpga_, DATA_SIZE);

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
RCLCPP_COMPONENTS_REGISTER_NODE(composition::DoubleVaddComponentFPGA)


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<composition::DoubleVaddComponentFPGA>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
