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

#include <fstream>
#include "adder.hpp"

int main(void) {
  // Setup some required variables
  int a, b;
  int result, expected_result, error_count = 0;

  // Test Loop
  for (int i=0 ; i < 10 ; i++) {
    // Setup inputs
    a = i;
    b = 100+(2*i);
    // Predict result
    expected_result = a*a*a + b*b;
    // Call DUT
    result = simple_adder(a, b);
    // Check result and output some visual check
    if (result != expected_result) {
      error_count++;
    }
    printf("Expected result: %d, Got Result: %d\n", expected_result, result);
  }
  return error_count;
}
