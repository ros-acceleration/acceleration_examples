cmake_minimum_required(VERSION 3.5)
project(faster_triplevadd_publisher)

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

# faster_vadd
add_executable(faster_vadd
  src/faster_triplevadd.cpp
  # src/vadd.cpp
)
target_include_directories(faster_vadd PUBLIC include)
target_link_libraries(faster_vadd
  ${OpenCL_LIBRARY}
  pthread
)
ament_target_dependencies(faster_vadd
  vitis_common
)


# faster_triplevadd_publisher
add_executable(faster_triplevadd_publisher
  src/faster_triplevadd_publisher.cpp
  # src/vadd.cpp
)
target_include_directories(faster_triplevadd_publisher PUBLIC include)
target_link_libraries(faster_triplevadd_publisher
  ${OpenCL_LIBRARY}
  pthread
)
ament_target_dependencies(faster_triplevadd_publisher
  rclcpp
  std_msgs
  vitis_common
)

if(ROS_ACCELERATION)
  # C simulation and synthesis
  vitis_hls_generate_tcl(
    PROJECT
      project_faster_triplevadd_publisher
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
    # RTLSIMULATION
    # RTLIMPLEMENTATION
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

install(
  TARGETS
    faster_vadd
    faster_triplevadd_publisher
  DESTINATION
    lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
