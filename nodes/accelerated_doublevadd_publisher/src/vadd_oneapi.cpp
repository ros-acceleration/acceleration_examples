#include <iostream>

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
  int len;          // Length of the arrays

  // Overloaded function operator to perform vector addition.
  void operator()() const {
    for (int idx = 0; idx < len; idx++) {
      int a_val = a_in[idx];
      int b_val = b_in[idx];
      int sum = a_val + b_val;
      c_out[idx] = sum;
    }
  }
};

// Define a constant size for the vectors.
constexpr int kVectSize = 256;

int main() {
  bool passed = false;

  try {
    // Use compile-time macros to determine which FPGA device to target:
    //  - FPGA emulator device (CPU emulation of the FPGA)
    //  - Actual FPGA device
    //  - FPGA simulator device
#if FPGA_SIMULATOR
    sycl::ext::intel::fpga_simulator_selector selector;
#elif FPGA_HARDWARE
    sycl::ext::intel::fpga_selector selector;
#else  // Assume FPGA emulator if neither of the above is defined
    sycl::ext::intel::fpga_emulator_selector selector;
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

    // Initialize arrays 'a' and 'b'.
    for (int i = 0; i < kVectSize; i++) {
      a[i] = i;
      b[i] = (kVectSize - i);
    }

    std::cout << "Adding two vectors of size " << kVectSize << std::endl;

    // Submit a task to the queue to execute the VectorAdd kernel.
    q.single_task<VectorAddID>(VectorAdd{a, b, c, kVectSize}).wait();

    // Verify that the result in array 'c' is correct.
    passed = true;
    for (int i = 0; i < kVectSize; i++) {
      int expected = a[i] + b[i];
      if (c[i] != expected) {
        std::cout << "idx=" << i << ": result " << c[i] << ", expected ("
                  << expected << ") A=" << a[i] << " + B=" << b[i] << std::endl;
        passed = false;
      }
    }

    // Print the verification result.
    std::cout << (passed ? "PASSED" : "FAILED") << std::endl;

    // Deallocate the arrays.
    sycl::free(a, q);
    sycl::free(b, q);
    sycl::free(c, q);

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
  return passed ? EXIT_SUCCESS : EXIT_FAILURE;
}
