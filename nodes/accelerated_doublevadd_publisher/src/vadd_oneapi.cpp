#include <iostream>
#include <cstdlib>

// Include the oneAPI headers for FPGA support.
#include <sycl/ext/intel/fpga_extensions.hpp>
#include <sycl/sycl.hpp>
#include "exception_handler.hpp" // Include the custom exception handler.

// Forward declare the kernel name in the global scope. This is an FPGA best
// practice to avoid issues related to name mangling in optimization reports.
class VectorAddID;

// Define a structure for Vector Addition.
struct VectorAdd {
  int *const a_in;  // Input array A
  int *const b_in;  // Input array B
  int *const c_out; // Output array (result of A + B)
  int len;               // Length of the arrays

  // // Overloaded function operator to perform vector addition.
  // void operator()() const {
  //   for (int idx = 0; idx < len; idx++) {
  //     int a_val = a_in[idx];
  //     int b_val = b_in[idx];
  //     int sum = a_val + b_val;
  //     c_out[idx] = sum;
  //   }
  // }

  // Overloaded function operator to perform double vector addition.
  void operator()() const {
    for (int j = 0; j < len; j++) {
      for (int i = 0; i < len; i++) {
        int a_val = a_in[i];
        int b_val = b_in[i];
        int sum = a_val + b_val;
        c_out[i] = sum;
      }
    }
  }
};

#define DATA_SIZE 4096  // 2**12
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

// Define a constant size for the vectors.
#define DATA_SIZE 4096  // 2**12
constexpr int kVectSize = DATA_SIZE;
// constexpr int kVectSize = 4096;

int main() {
  bool passed = false;
  int n = 0;

  try {
    // Use compile-time macros to determine which FPGA device to target:
    //  - FPGA emulator device (CPU emulation of the FPGA)
    //  - Actual FPGA device
    //  - FPGA simulator device
// #if FPGA_SIMULATOR
//     sycl::ext::intel::fpga_simulator_selector selector;
// #elif FPGA_HARDWARE
//     sycl::ext::intel::fpga_selector selector;
// #else  // Assume FPGA emulator if neither of the above is defined
//     sycl::ext::intel::fpga_emulator_selector selector;
// #endif

// OneAPI 2023.0
#if FPGA_SIMULATOR
    auto selector = sycl::ext::intel::fpga_simulator_selector{};
#elif FPGA_HARDWARE
    auto selector = sycl::ext::intel::fpga_selector{};
#else  // #if FPGA_EMULATOR
    auto selector = sycl::ext::intel::fpga_emulator_selector{};
#endif

    // Create a SYCL queue with the selected device, custom exception handler, and enable profiling.
    sycl::queue q(selector, fpga_tools::exception_handler,
                  sycl::property::queue::enable_profiling{});

    // Fetch the device associated with the queue.
    auto device = q.get_device();

    // Ensure the selected device supports Unified Shared Memory (USM) host allocations.
    if (!device.has(sycl::aspect::usm_host_allocations)) {
      std::cerr << "This design must either target a board that supports USM "
                   "Host/Shared allocations, or IP Component Authoring. "
                << std::endl;
      std::terminate();
    }

    // Print the device name.
    std::cout << "Running on device: "
              << device.get_info<sycl::info::device::name>().c_str()
              << std::endl;

    // Allocate arrays in shared memory so the kernel can access them.
    int *a = sycl::malloc_shared<int>(kVectSize, q);
    int *b = sycl::malloc_shared<int>(kVectSize, q);
    int *c = sycl::malloc_shared<int>(kVectSize, q);

    // Application variables
    // int in1[DATA_SIZE];
    // int in2[DATA_SIZE];
    // int out[DATA_SIZE];
    int *in1 = sycl::malloc_shared<int>(DATA_SIZE, q);
    int *in2 = sycl::malloc_shared<int>(DATA_SIZE, q);
    int *out = sycl::malloc_shared<int>(DATA_SIZE, q);

    // size_t length = DATA_SIZE;
    // Init out to zeros
    for (int i = 0 ; i < DATA_SIZE ; i++) {
        out[i] = 0;
    }

    while (1) {
        
        // Option 1: Similar to the ROS Node
        // randomize the vectors used
        for (int i = 0 ; i < DATA_SIZE ; i++) {
            in1[i] = rand() % DATA_SIZE;  // NOLINT
            in2[i] = rand() % DATA_SIZE;  // NOLINT
            out[i] = 0;
        }
        std::cout << "Adding two vectors of size " << kVectSize << ", iteration: " << n << std::endl;
        n++;        
        // Submit a task to the queue to execute the VectorAdd kernel.
        q.single_task<VectorAddID>(VectorAdd{in1, in2, out, kVectSize}).wait();
        // Validate operation
        check_vadd(in1, in2, out);

        // // Option 2: Originally proposed by SYCL examples
        // // Initialize arrays 'a' and 'b'.
        // for (int i = 0; i < kVectSize; i++) {
        //   a[i] = i;
        //   b[i] = (kVectSize - i);
        // }

        // std::cout << "Adding two vectors of size " << kVectSize << ", iteration: " << n << std::endl;
        // n++;

        // // Submit a task to the queue to execute the VectorAdd kernel.
        // q.single_task<VectorAddID>(VectorAdd{a, b, c, kVectSize}).wait();

        // // Verify that the result in array 'c' is correct.
        // passed = true;
        // for (int i = 0; i < kVectSize; i++) {
        //   int expected = a[i] + b[i];
        //   if (c[i] != expected) {
        //     std::cout << "idx=" << i << ": result " << c[i] << ", expected ("
        //               << expected << ") A=" << a[i] << " + B=" << b[i] << std::endl;
        //     passed = false;
        //   }
        // }        
    }  

  } catch (sycl::exception const &e) {
    // Handle exceptions that might arise in the host code.
    std::cerr << "Caught a SYCL host exception:\n" << e.what() << "\n";

    // If an FPGA hardware is not found, give some helpful debugging info.
    if (e.code().value() == CL_DEVICE_NOT_FOUND) {
      std::cerr << "If you are targeting an FPGA, ensure your system has a "
                   "correctly configured FPGA board.\n";
      std::cerr << "Run sys_check in the oneAPI root directory to verify.\n";
      std::cerr << "If targeting the FPGA emulator, compile with -DFPGA_EMULATOR.\n";
    }
    std::terminate();
  }

  // Return success or failure.
  return 0;
}
