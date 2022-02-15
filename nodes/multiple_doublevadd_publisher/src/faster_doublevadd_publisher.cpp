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

// #define DATA_SIZE 512  // 2**9
#define DATA_SIZE 4096  // 2**12
// #define DATA_SIZE 16384  // 2**14
// #define DATA_SIZE 65536  // 2**16
// #define DATA_SIZE 262144  // 2**18

#include <chrono>  // NOLINT
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "tracetools/tracetools.h"
#include "tracetools_acceleration/tracetools.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <vitis_common/common/ros_opencl_120.hpp>
#include <vitis_common/common/utilities.hpp>

using namespace std::chrono_literals;  // NOLINT

/**
 * @brief Check that the vector add operation
 *      was successfully computed, either in PL or PS.
 *
 * @param in1 summatory operand 1
 * @param in2 summatory operand 1
 * @param out result of the summatory
 * @return true if successful
 * @return false if failed
 */
bool check_vadd(
          const int *in1,  // Read-Only Vector 1
          const int *in2,  // Read-Only Vector 2
          const int *out   // Read-Only Result
    ) {
  bool match = true;
  for (int i = 0 ; i < DATA_SIZE ; i++) {
      int expected = in1[i] + in2[i];
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
  auto node = rclcpp::Node::make_shared("faster_doublevadd_publisher");
  auto publisher = node->create_publisher<std_msgs::msg::String>("vector_acceleration", 10);
  auto publish_count = 0;
  std_msgs::msg::String message;
  rclcpp::WallRate loop_rate(100ms);

  // ------------------------------------------------------------------------
  // Step 1: Initialize the OpenCL environment for acceleration
  // ------------------------------------------------------------------------
  cl_int err;
  std::string binaryFile = (argc < 2) ? "vadd.xclbin" : argv[1];
  unsigned fileBufSize;
  std::vector<cl::Device> devices = get_xilinx_devices();
  devices.resize(1);
  cl::Device device = devices[0];
  cl::Context context(device, NULL, NULL, NULL, &err);
  char* fileBuf = read_binary_file(binaryFile, fileBufSize);
  cl::Program::Binaries bins{{fileBuf, fileBufSize}};
  cl::Program program(context, devices, bins, NULL, &err);
  cl::CommandQueue q(context, device, CL_QUEUE_PROFILING_ENABLE, &err);
  cl::Kernel krnl_vector_add(program, "vadd_faster", &err);

  // ------------------------------------------------------------------------
  // Step 2: Create buffers, map memory
  // ------------------------------------------------------------------------
  // Create the buffers and allocate memory
  cl::Buffer in1_buf(context, CL_MEM_ALLOC_HOST_PTR | CL_MEM_READ_ONLY,
    sizeof(int) * DATA_SIZE, NULL, &err);
  cl::Buffer in2_buf(context, CL_MEM_ALLOC_HOST_PTR | CL_MEM_READ_ONLY,
    sizeof(int) * DATA_SIZE, NULL, &err);
  cl::Buffer out_buf(context, CL_MEM_ALLOC_HOST_PTR | CL_MEM_WRITE_ONLY,
    sizeof(int) * DATA_SIZE, NULL, &err);

  // Map buffers to kernel arguments, thereby assigning
  //  them to specific device memory banks
  krnl_vector_add.setArg(0, in1_buf);
  krnl_vector_add.setArg(1, in2_buf);
  krnl_vector_add.setArg(2, out_buf);

  // Map host-side buffer memory to user-space pointers
  int *in1 = (int *)q.enqueueMapBuffer(in1_buf, CL_TRUE, CL_MAP_WRITE, 0, sizeof(int) * DATA_SIZE);  // NOLINT
  int *in2 = (int *)q.enqueueMapBuffer(in2_buf, CL_TRUE, CL_MAP_WRITE, 0, sizeof(int) * DATA_SIZE);  // NOLINT
  int *out = (int *)q.enqueueMapBuffer(out_buf, CL_TRUE, CL_MAP_WRITE | CL_MAP_READ, 0, sizeof(int) * DATA_SIZE);  // NOLINT

  // ------------------------------------------------------------------------
  // Step 3: Main loop, set kernel arguments, schedule transfer
  //  of memory to kernel, run kernel and transfer memory back from it
  // ------------------------------------------------------------------------
  while (rclcpp::ok()) {
    // randomize the vectors used
    for (int i = 0 ; i < DATA_SIZE ; i++) {
        in1[i] = rand() % DATA_SIZE;  // NOLINT
        in2[i] = rand() % DATA_SIZE;  // NOLINT
        out[i] = 0;
    }

    TRACEPOINT(vadd_pre, ("iteration: " + std::to_string(publish_count)).c_str());
    // Set kernel arguments
    krnl_vector_add.setArg(0, in1_buf);
    krnl_vector_add.setArg(1, in2_buf);
    krnl_vector_add.setArg(2, out_buf);
    krnl_vector_add.setArg(3, DATA_SIZE);

    // Schedule transfer of inputs to device memory
    q.enqueueMigrateMemObjects({in1_buf, in2_buf}, 0 /* 0 means from host*/);
    // execution of kernel
    q.enqueueTask(krnl_vector_add);
    // transfer of outputs back to host memory
    q.enqueueMigrateMemObjects({out_buf}, CL_MIGRATE_MEM_OBJECT_HOST);
    // Wait for all scheduled operations to finish
    q.finish();
    TRACEPOINT(vadd_post, ("iteration: " + std::to_string(publish_count)).c_str());

    // Validate operation in the PS
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
  delete[] fileBuf;  // release memory from the acceleration kernel

  return 0;
}
