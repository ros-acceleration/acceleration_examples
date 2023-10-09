cmake_minimum_required(VERSION 3.5)
project(accelerated_vadd_publisher)

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
find_package(ament_oneapi)
find_package(accelerated_doublevadd_publisher REQUIRED)

if (ROS_ACCELERATION)
  if (ROS_ONEAPI)
    find_package(tracetools_acceleration REQUIRED)

    # accelerated_vadd_publisher_agilex
    add_executable(accelerated_vadd_publisher_agilex
      src/accelerated_vadd_publisher_agilex.cpp
    )
    target_include_directories(accelerated_vadd_publisher_agilex PUBLIC include)
    target_link_libraries(accelerated_vadd_publisher_agilex
      # avalonmm_driver
      pthread
      accelerated_doublevadd_publisher::avalonmm_driver
    )
    ament_target_dependencies(accelerated_vadd_publisher_agilex
      rclcpp
      std_msgs
      tracetools 
      tracetools_acceleration
      accelerated_doublevadd_publisher
    )

    # vadd kernel
    oneapi_acceleration_kernel(
      NAME accelerated_vadd
      FILE src/vadd_oneapi.cpp
      FPGA_EMIF_ENABLED 0
      FPGA_EMIF_CTRL_BASE_ADDRESS 0x4000
      FPGA_EMIF_HOST_ADDR_WIDTH 31
      FPGA_EMIF_AGENT_ADDR_WIDTH 27
      AVMM_HOST auto
      INCLUDE
        include
      TYPE
        fpga_emu
        fpga_sim
        report
        fpga_ip_export
      USER_SUBSYSTEM src/sed_vadd_user_subsystem.tcl
    )

    install(
      TARGETS
        # kernel_driver
        accelerated_vadd_publisher_agilex
      DESTINATION 
        lib/${PROJECT_NAME}
    )
  endif()  # ROS_ONEAPI
endif()  # ROS_ACCERATION

ament_package()
