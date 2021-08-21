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

#include <chrono>  // NOLINT
#include <functional>
#include <memory>
#include <string>

#include "vadd.hpp"

// #define DATA_SIZE 512  // 2**9
#define DATA_SIZE 4096  // 2**12
// #define DATA_SIZE 16384  // 2**14
// #define DATA_SIZE 65536  // 2**16
// #define DATA_SIZE 262144  // 2**18

// using namespace std::chrono_literals;  // NOLINT

int check_vadd(
          const unsigned int *in1,  // Read-Only Vector 1
          const unsigned int *in2,  // Read-Only Vector 2
          const unsigned int *out   // Read-Only Result
    ) {
  // bool match = true;
  // no need to iterate twice through the loop, math's the same
  for (int i = 0 ; i < DATA_SIZE ; i++) {
      unsigned int expected = in1[i]+in2[i];
      if (out[i] != expected) {
          // std::cout << "Error: Result mismatch" << std::endl;
          // std::cout << "i = " << i << " CPU result = "
          //   << expected << " Device result = " << out[i] << std::endl;
          // match = false;
          // break;
          return out[i] - expected;
      }
  }
  return 0;
  // return match;
}

int main(int argc, char * argv[]) {

  // Application variables
  unsigned int in1[DATA_SIZE];
  unsigned int in2[DATA_SIZE];
  unsigned int out[DATA_SIZE];

  // randomize the vectors used
  for (int i = 0 ; i < DATA_SIZE ; i++) {
      in1[i] = rand() % DATA_SIZE;  // NOLINT
      in2[i] = rand() % DATA_SIZE;  // NOLINT
      out[i] = 0;
  }

  // Add vectors
  vadd(in1, in2, out, DATA_SIZE);  // function subject to be accelerated

  // Validate operation
  return check_vadd(in1, in2, out);
}
