cmake_minimum_required(VERSION 3.5)
project(multiple_doublevadd_publisher)

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
find_package(Threads REQUIRED)
find_package(tracetools_acceleration REQUIRED)
find_package(ament_vitis)

if (ROS_VITIS)
  find_package(OpenCL REQUIRED)
  find_package(vitis_common REQUIRED)
  set (DEPS
        rclcpp
        std_msgs
        vitis_common
        tracetools
        tracetools_acceleration)

  # faster_doublevadd_publisher
  add_executable(faster_doublevadd_publisher src/faster_doublevadd_publisher.cpp)
  target_link_libraries(faster_doublevadd_publisher ${OpenCL_LIBRARY} pthread)
  ament_target_dependencies(faster_doublevadd_publisher ${DEPS})

  # accelerated_doublevadd_publisher
  add_executable(accelerated_doublevadd_publisher src/accelerated_doublevadd_publisher.cpp)
  target_link_libraries(accelerated_doublevadd_publisher ${OpenCL_LIBRARY} pthread)
  ament_target_dependencies(accelerated_doublevadd_publisher ${DEPS})

  # offloaded_doublevadd_publisher
  add_executable(offloaded_doublevadd_publisher src/offloaded_doublevadd_publisher.cpp)
  target_link_libraries(offloaded_doublevadd_publisher ${OpenCL_LIBRARY} pthread)
  ament_target_dependencies(offloaded_doublevadd_publisher ${DEPS})

  # acceleration kernels
  if(ROS_ACCELERATION)

    # 1. synthesize xilinx objects (.xo)
    # 1.1 vadd_faster kernel
    vitis_acceleration_kernel(
      NAME vadd_faster
      FILE src/vadd_faster.cpp
      CONFIG src/kv260.cfg
      CLOCK 100000000:vadd_faster
      INCLUDE include
      TYPE hw
    )

    # 1.2 vadd_accelerated kernel
    vitis_acceleration_kernel(
      NAME vadd_accelerated
      FILE src/vadd_accelerated.cpp
      CONFIG src/kv260.cfg
      CLOCK 100000000:vadd_accelerated
      INCLUDE include
      TYPE hw
    )

    # 1.3 vadd_offloaded kernel
    vitis_acceleration_kernel(
      NAME vadd_offloaded
      FILE src/vadd_offloaded.cpp
      CONFIG src/kv260.cfg
      CLOCK 100000000:vadd_offloaded
      INCLUDE include
      TYPE hw
    )

    # 2. place and route all kernels together (link)
    vitis_link_kernel(
      OUTPUT multiple
      KERNELS vadd_faster vadd_accelerated vadd_offloaded
      CONFIG src/kv260_multiple.cfg
    )
  endif()

  install(
    TARGETS
      faster_doublevadd_publisher
      accelerated_doublevadd_publisher
      offloaded_doublevadd_publisher
    DESTINATION
      lib/${PROJECT_NAME}
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
endif()  # ROS_VITIS

ament_package()
