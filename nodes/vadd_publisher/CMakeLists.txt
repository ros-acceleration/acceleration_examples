cmake_minimum_required(VERSION 3.5)
project(vadd_publisher)

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
find_package(tracetools_acceleration REQUIRED)

add_executable(vadd_publisher src/vadd_publisher.cpp src/vadd.cpp)
target_include_directories(vadd_publisher PUBLIC include)
ament_target_dependencies(vadd_publisher rclcpp std_msgs tracetools tracetools_acceleration)

if(ROS_ACCELERATION)
  if (ROS_VITIS)
    # C simulation and synthesis
    vitis_hls_generate_tcl(
      PROJECT
        project_vadd_publisher
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
  endif()  # ROS_VITIS
  if (ROS_ONEAPI)    

  endif()  # ROS_ONEAPI
endif()  # ROS_ACCELERATION

install(TARGETS
  vadd_publisher
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
