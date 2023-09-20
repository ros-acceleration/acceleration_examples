//    @@@@@@@@@@@@@@@@@@@@
//    @@@@@@@@@&@@@&&@@@@@
//    @@@@@ @@  @@    @@@@
//    @@@@@ @@  @@    @@@@
//    @@@@@ @@  @@    @@@@ Copyright (c) 2023, Acceleration Robotics®
//    @@@@@ @@  @@    @@@@ Author: Víctor Mayoral Vilches <victor@accelerationrobotics.com>
//    @@@@@ @@  @@    @@@@
//    @@@@@@@@@&@@@@@@@@@@
//    @@@@@@@@@@@@@@@@@@@@

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. 

#include <climits>
#include <cstdint>
#include <iostream>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include "avalonmm_driver.hpp"
#include <chrono>  // NOLINT
#include <functional>
#include <memory>
#include <string>

// #include <lttng/tracef.h>  // uncoment if using tracef
#include "tracetools/tracetools.h"
#include "tracetools_acceleration/tracetools.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "vadd.hpp"

#define ZTS11VECTORADDID_REGISTER_MAP_OFFSET (0x80000)
#include "ZTS11VectorAddID_register_map.hpp"
// #define DATA_SIZE 3  
// #define DATA_SIZE 256
#define DATA_SIZE 4096  // 2**12
// #define DATA_SIZE 16384  // 2**14
// #define DATA_SIZE 65536  // 2**16
// #define DATA_SIZE 262144  // 2**18

using namespace std::chrono_literals;  // NOLINT

bool check_vadd(
          const int *in1,  // Read-Only Vector 1
          const int *in2,  // Read-Only Vector 2
          const int *out   // Read-Only Result
    ) {
  bool match = true;
  // no need to iterate twice through the loop, math's the same
  for (int i = 0 ; i < DATA_SIZE ; i++) {
      int expected = in1[i]+in2[i];
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
    auto node = rclcpp::Node::make_shared("accelerated_doublevadd_publisher");
    auto publisher = node->create_publisher<std_msgs::msg::String>("vector", 10);
    auto publish_count = 0;
    std_msgs::msg::String message;
    rclcpp::WallRate loop_rate(100ms);

    // AvalonMM_Driver for SYCL kernel interactions
    constexpr uintptr_t KERNEL_DRIVER_MMMIO_BASE_ADDRESS = 0xF9000000;
    constexpr size_t MMIO_REGION_SIZE = 0x90000;
    // These offsets are determined with respect KERNEL_DRIVER_MMMIO_BASE_ADDRESS
    // and should be within the MMIO_REGION_SIZE size

    enum MemoryOffsets {
        ARG_A_OFFSET = 0x0,
        ARG_B_OFFSET = ARG_A_OFFSET + DATA_SIZE * sizeof(uint64_t),
        ARG_C_OFFSET = ARG_B_OFFSET + DATA_SIZE * sizeof(uint64_t),
        ARG_LEN_OFFSET = ARG_C_OFFSET + DATA_SIZE * sizeof(uint64_t),
    };

    std::cout << "ARG_A_OFFSET: 0x" << std::hex << ARG_A_OFFSET << std::endl;
    std::cout << "ARG_B_OFFSET: 0x" << std::hex << ARG_B_OFFSET << std::endl;
    std::cout << "ARG_C_OFFSET: 0x" << std::hex << ARG_C_OFFSET << std::endl;
    std::cout << "ARG_LEN_OFFSET: 0x" << std::hex << ARG_LEN_OFFSET << std::endl;

    AvalonMM_Driver driver(KERNEL_DRIVER_MMMIO_BASE_ADDRESS, MMIO_REGION_SIZE);

    // Application variables
    int in1[DATA_SIZE];
    int in2[DATA_SIZE];
    int out[DATA_SIZE];
    size_t length = DATA_SIZE;
    
    // Init out to zeros
    for (int i = 0 ; i < DATA_SIZE ; i++) {
        out[i] = 0;
    }
    /// arg_c
    driver.set_arg(
        ZTS11VECTORADDID_REGISTER_MAP_ARG_ARG_C_OUT_REG, 
        ARG_C_OFFSET, 
        out, 
        length);

    /// set the len argument, which won't change over time
    // NOTE that the need to change size_t to int* to use "length"
    // as argument for values is due to the fact that the AvalonMM_Driver
    if (length > INT_MAX) {
        // Handle overflow
        printf("The value is too large to fit in an int.\n");
        exit(1);
    }
    int intLength = (int)length;
    int* intPointer = &intLength;

    driver.set_arg(
        ZTS11VECTORADDID_REGISTER_MAP_ARG_ARG_LEN_REG, 
        ARG_LEN_OFFSET, 
        intPointer,
        1);

    while (rclcpp::ok()) {
        // randomize the vectors used
        for (int i = 0 ; i < DATA_SIZE ; i++) {
            in1[i] = rand() % DATA_SIZE;  // NOLINT
            in2[i] = rand() % DATA_SIZE;  // NOLINT
            out[i] = 0;  // NOTE: this could be ommitted if the kernel is expected to write to all the memory
        }

        // Set kernel arguments and initiate kernel execution.
        /// arg_a
        driver.set_arg(
            ZTS11VECTORADDID_REGISTER_MAP_ARG_ARG_A_IN_REG, 
            ARG_A_OFFSET, 
            in1, 
            length);

        /// arg_b
        driver.set_arg(
            ZTS11VECTORADDID_REGISTER_MAP_ARG_ARG_B_IN_REG, 
            ARG_B_OFFSET, 
            in2, 
            length);

        // Add vectors
        // tracef("ros2:krs:doublevadd_publisher:main calling vadd");
        TRACEPOINT(vadd_pre, ("iteration: " + std::to_string(publish_count)).c_str());
        // vadd(in1, in2, out, DATA_SIZE);  // function subject to be accelerated

        driver.start_kernel(ZTS11VECTORADDID_REGISTER_MAP_STATUS_REG, KERNEL_REGISTER_MAP_GO_MASK);
        while (!driver.is_kernel_done(ZTS11VECTORADDID_REGISTER_MAP_STATUS_REG, KERNEL_REGISTER_MAP_DONE_MASK)) {
            // Waiting for the kernel to complete
        }

        ////// Debug
        int finish_counter1 = driver.read_register(ZTS11VECTORADDID_REGISTER_MAP_FINISHCOUNTER_REG1);
        int finish_counter2 = driver.read_register(ZTS11VECTORADDID_REGISTER_MAP_FINISHCOUNTER_REG2);
        // std::cout << "Kernel finish counter 1: " << finish_counter1 << std::endl;
        // std::cout << "Kernel finish counter 2: " << finish_counter2 << std::endl;
        ////// Debug

        driver.get_arg(
            ARG_C_OFFSET,
            out,
            length);

        // ////// Debug
        // driver.dump_arg_values(ARG_A_OFFSET, length);
        // driver.dump_arg_values(ARG_B_OFFSET, length);
        // driver.dump_arg_values(ARG_C_OFFSET, length);

        // std::cout << "out = ";
        // for (size_t i = 0; i < DATA_SIZE; ++i) {
        //     std::cout << "0x" << std::hex << out[i];
        //     if (i != DATA_SIZE - 1) {
        //         std::cout << " ";
        //     }
        // }
        // std::cout << std::dec << std::endl;  // Switch back to decimal mode for any subsequent output
        ////// Debug

        TRACEPOINT(vadd_post, ("iteration: " + std::to_string(publish_count)).c_str());

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
