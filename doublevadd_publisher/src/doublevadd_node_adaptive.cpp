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
that demonstrates the concept of a ROS 2 Adaptive Node/Components through
a ROS 2 "adaptive" parameter, which determines if computations happen in the
CPU or in the FPGA, adaptively.

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

class DoubleVaddNodeAdaptive : public rclcpp::Node
{
public:
  DoubleVaddNodeAdaptive(const rclcpp::NodeOptions & options)
  : Node("doublevadd_publisher", options), count_(0)
  {
    // "adaptive" parameter,
    //    0: CPU-based computations,
    //    1: FPGA-offloaded computations
    const char * param_name = "adaptive";
    this->declare_parameter<int>(param_name, 0);

    // Create a publisher of "std_mgs/String" messages on the "chatter" topic.
    pub_ = create_publisher<std_msgs::msg::String>("vector", 10);

    // define differential time value (in ms) for the loop
    dt_ = 100ms;  // 10 Hz

    // Use a timer to schedule periodic message publishing.
    timer_ = create_wall_timer(dt_, std::bind(&DoubleVaddNodeAdaptive::on_timer, this));

    // Fetch arguments from rcl
    auto context = options.context();
    auto rcl_context = context->get_rcl_context();
    argc_ = rcl_context->impl->argc;
    argv_ = rcl_context->impl->argv;

    // Parameter callback
    // NOTE: not available in Foxy
    //
    // TODO: re-evaluate in the future, see
    // https://github.com/ros2/demos/blob/master/demo_nodes_cpp/src/parameters/parameter_event_handler.cpp#L83-L91
    //
    // // Now, create a parameter subscriber that can be used to monitor parameter changes on
    // // our own local node as well as other remote nodes
    // auto param_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);
    //
    // // First, set a callback for the local integer parameter. In this case, we don't
    // // provide a node name (the third, optional, parameter).
    // auto cb1 = [this](const rclcpp::Parameter & p) {
    //     RCLCPP_INFO(
    //       this->get_logger(),
    //       "cb1: Received an update to parameter \"%s\" of type %s: \"%" PRId64 "\"",
    //       p.get_name().c_str(),
    //       p.get_type_name().c_str(),
    //       p.as_int());
    //   };
    // auto handle1 = param_subscriber->add_parameter_callback(param_name, cb1);

  }  // DoubleVaddNodeAdaptive constructor

  ~DoubleVaddNodeAdaptive()
  {
    // Free memory for OpenCL buffers
    delete in1_buf_;
    delete in2_buf_;
    delete out_buf_;

    // Free memory for other OpenCL constructs
    delete q_;
    delete krnl_vector_add_;

  }  // DoubleVaddNodeAdaptive destructor

protected:
  /// `init_fpga`
  /**
   * Initialize memory and OpenCL abstractions for interacting with the FPGA
   */
  void init_fpga()
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
  }

  /// `compute_cpu`
  /**
   * Perform vector-add Node computations within the CPU.
   *
   * \param[in] in1, vector array operand 1
   * \param[in] in2, vector array operand 2
   * \param[out] out, vector array with the resulting addition
   * \param[in] size, size of the vectors
   */
  void compute_cpu(
          const unsigned int *in1,  // Read-Only Vector 1
          const unsigned int *in2,  // Read-Only Vector 2
          unsigned int *out,        // Output Result
          int size)                 // Size in integer
  {
    // Add vectors
    // tracef("ros2:krs:doublevadd_publisher:main calling vadd");
    TRACEPOINT(vadd_pre, ("iteration: " + std::to_string(publish_count_)).c_str());
    vadd(in1, in2, out, size);  // function subject to be accelerated
    TRACEPOINT(vadd_post, ("iteration: " + std::to_string(publish_count_)).c_str());

    // Validate operation
    check_compute(in1, in2, out, size);
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
  void compute_fpga(
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
  bool check_compute(
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

  void on_timer()
  {
    // TODO: move to parameter callbacks from
    //  Galactic on
    this->get_parameter("adaptive", adaptive_value_);
    unsigned int *in1;
    unsigned int *in2;
    unsigned int *out;
    if (adaptive_value_ == 1) {
      // Initialize FPGA if appropriate
      if (!fpga_initialized_) {
        init_fpga();
        fpga_initialized_ = true;
      }
      // RCLCPP_INFO(this->get_logger(), "compute_fpga");
      in1 = in1_fpga_;
      in2 = in2_fpga_;
      out = out_fpga_;
    } else if (adaptive_value_ == 0) {
      // RCLCPP_INFO(this->get_logger(), "compute_cpu");
      in1 = in1_;
      in2 = in2_;
      out = out_;
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "Invalid 'adaptive' parameter value");
    }

    // randomize the vectors used
    for (int i = 0 ; i < DATA_SIZE ; i++) {
        in1[i] = rand() % DATA_SIZE;  // NOLINT
        in2[i] = rand() % DATA_SIZE;  // NOLINT
        out[i] = 0;
    }

    if (adaptive_value_ == 1) {
      compute_fpga(in1, in2, out, DATA_SIZE);
    } else if (adaptive_value_ == 0) {
      compute_cpu(in1, in2, out, DATA_SIZE);
    }

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

private:
  size_t count_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::milliseconds dt_;

  // Buffers for CPU operation
  // TODO: check if these can be replaced by in1_fpga_, etc.
  unsigned int in1_[DATA_SIZE];
  unsigned int in2_[DATA_SIZE];
  unsigned int out_[DATA_SIZE];

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

  int publish_count_ = 0;
  bool fpga_initialized_ = false;

  int argc_;
  char **argv_;
  int adaptive_value_;

};  // DoubleVaddNodeAdaptive class

}  // namespace composition

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(composition::DoubleVaddNodeAdaptive)


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<composition::DoubleVaddNodeAdaptive>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
