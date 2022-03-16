cmake_minimum_required(VERSION 3.5)
project(offloaded_doublevadd_publisher)

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
find_package(ament_vitis)

if (ROS_VITIS)
  find_package(vitis_common REQUIRED)
  find_package(OpenCL REQUIRED)

  # offloaded_doublevadd_publisher
  add_executable(offloaded_doublevadd_publisher
    src/offloaded_doublevadd_publisher.cpp
    src/vadd.cpp
  )
  target_include_directories(offloaded_doublevadd_publisher PUBLIC include)
  target_link_libraries(offloaded_doublevadd_publisher
    ${OpenCL_LIBRARY}
    pthread
  )
  ament_target_dependencies(offloaded_doublevadd_publisher
    rclcpp
    std_msgs
    vitis_common
  )

  if(ROS_ACCELERATION)
    # C simulation and synthesis
    vitis_hls_generate_tcl(
      PROJECT
        project_offloaded_doublevadd_publisher
      SRC
        src/vadd.cpp
      HEADERS
        include
      TESTBENCH
        src/testbench.cpp
      TOPFUNCTION
        vadd_offloaded
      CLOCK
        4
      SYNTHESIS
    )


    # vadd kernel
    vitis_acceleration_kernel(
      NAME vadd_offloaded
      FILE src/vadd.cpp
      CONFIG src/kv260.cfg
      INCLUDE
        include
      TYPE
        # sw_emu
        # hw_emu
        hw
      LINK
      PACKAGE
    )
  endif()

  install(TARGETS
    offloaded_doublevadd_publisher
    DESTINATION lib/${PROJECT_NAME}
  )

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()
endif()  # ROS_VITIS

ament_package()
