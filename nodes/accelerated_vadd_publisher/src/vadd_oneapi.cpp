#include <iostream>
#include <cstdlib>

// Include the oneAPI headers for FPGA support.
#include <sycl/ext/intel/fpga_extensions.hpp>
#include <sycl/sycl.hpp>
#include "exception_handler.hpp" // Include the custom exception handler.

// Forward declare the kernel name in the global scope. This is an FPGA best
// practice to avoid issues related to name mangling in optimization reports.
class VectorAddID;

struct VectorAdd {
  int *const a_in;
  int *const b_in;
  int *const c_out;
  const int len;

  void operator()() const {    
    [[intel::unroll(len)]]
    for (int i = 0; i < len; i++) {
      c_out[i] = a_in[i] + b_in[i];
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
  }  

  // Return success or failure.
  return 0;
}

