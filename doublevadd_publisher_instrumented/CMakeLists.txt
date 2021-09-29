cmake_minimum_required(VERSION 3.5)
project(doublevadd_publisher_instrumented)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unknown-pragmas)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_vitis)
find_package(tracetools REQUIRED)

add_executable(doublevadd_publisher_instrumented src/doublevadd_publisher.cpp src/vadd.cpp)
target_include_directories(doublevadd_publisher_instrumented PUBLIC include)
ament_target_dependencies(doublevadd_publisher_instrumented rclcpp std_msgs tracetools)

# C simulation and synthesis
vitis_hls_generate_tcl(
  PROJECT
    project_doublevadd_publisher_instrumented
  SRC
    src/vadd.cpp
  HEADERS
    include
  TESTBENCH
    src/testbench.cpp
  TOPFUNCTION
    vadd
  CLOCK
    4
  SYNTHESIS
)

install(TARGETS
  doublevadd_publisher_instrumented
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
