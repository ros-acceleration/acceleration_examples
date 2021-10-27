cmake_minimum_required(VERSION 3.5)
project(triplevadd_publisher)

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

add_executable(triplevadd_publisher src/triplevadd_publisher.cpp src/vadd.cpp)
target_include_directories(triplevadd_publisher PUBLIC include)
ament_target_dependencies(triplevadd_publisher rclcpp std_msgs)

if(ROS_ACCELERATION)
  # C simulation and synthesis
  vitis_hls_generate_tcl(
    PROJECT
      project_triplevadd_publisher
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
endif()

install(TARGETS
  triplevadd_publisher
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
