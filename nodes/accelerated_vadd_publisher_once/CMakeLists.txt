cmake_minimum_required(VERSION 3.5)
project(accelerated_vadd_publisher_once)

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
find_package(OpenCL REQUIRED)
find_package(Threads REQUIRED)
find_package(vitis_common REQUIRED)
find_package(ament_vitis)

# accelerated_vadd_publisher
add_executable(accelerated_vadd_publisher_once
  src/accelerated_vadd_publisher_once.cpp
  # src/vadd.cpp
)
target_include_directories(accelerated_vadd_publisher_once PUBLIC
  include
  # $ENV{XILINX_HLS}/common/technology/autopilot
)
target_link_libraries(accelerated_vadd_publisher_once
  ${OpenCL_LIBRARY}
  pthread
)
ament_target_dependencies(accelerated_vadd_publisher_once
  rclcpp
  std_msgs
)

if(ROS_ACCELERATION)
  # Generate Tcl script for analysis
  vitis_hls_generate_tcl(
    PROJECT
      project_accelerated_vadd_publisher_once
    SRC
      src/vadd.cpp
    HEADERS
      include
    TESTBENCH
      src/testbench_vadd.cpp
    TOPFUNCTION
      vadd
    CLOCK
      4
    SYNTHESIS
  )

  # vadd kernel
  vitis_acceleration_kernel(
    NAME vadd
    FILE src/vadd.cpp
    CONFIG src/kv260.cfg
    INCLUDE
      include
    TYPE
      # sw_emu
      # hw_emu
      hw
    PACKAGE
  )
endif()

install(TARGETS
  accelerated_vadd_publisher_once
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
