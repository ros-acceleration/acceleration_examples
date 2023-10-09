cmake_minimum_required(VERSION 3.5)
project(accelerated_doublevadd_publisher)

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

if (ROS_ACCELERATION)
  if (ROS_VITIS)
    find_package(vitis_common REQUIRED)
    find_package(OpenCL REQUIRED)

    # accelerated_doublevadd_publisher
    add_executable(accelerated_doublevadd_publisher
      src/accelerated_doublevadd_publisher.cpp
      src/vadd.cpp
    )
    target_include_directories(accelerated_doublevadd_publisher PUBLIC include)
    target_link_libraries(accelerated_doublevadd_publisher
      ${OpenCL_LIBRARY}
      pthread
    )
    ament_target_dependencies(accelerated_doublevadd_publisher
      rclcpp
      std_msgs
      vitis_common
    )

    if(ROS_ACCELERATION)
      # C simulation and synthesis
      vitis_hls_generate_tcl(
        PROJECT
          project_accelerated_doublevadd_publisher
        SRC
          src/vadd.cpp
        HEADERS
          include
        TESTBENCH
          src/testbench.cpp
        TOPFUNCTION
          vadd_accelerated
        CLOCK
          4
        SYNTHESIS
      )

      # vadd kernel
      vitis_acceleration_kernel(
        NAME vadd_accelerated
        FILE src/vadd.cpp
        CONFIG src/kv260.cfg
        # CONFIG src/zcu102.cfg
        CLOCK 100000000:vadd_accelerated
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
      accelerated_doublevadd_publisher
      DESTINATION lib/${PROJECT_NAME}
    )

    if(BUILD_TESTING)
      find_package(ament_lint_auto REQUIRED)
      ament_lint_auto_find_test_dependencies()
    endif()
  endif()  # ROS_VITIS


  if (ROS_ONEAPI)
    find_package(tracetools_acceleration REQUIRED)

    # # kernel_driver
    # # NOTE: only for prototyping purposes
    #
    # add_executable(kernel_driver
    #   src/kernel_driver.cpp
    # )
    # target_include_directories(kernel_driver PUBLIC include)
    # target_link_libraries(kernel_driver
    #   pthread
    # )
    # ament_target_dependencies(kernel_driver
    #   rclcpp
    #   std_msgs
    # )

    # Create avalonmm_driver library
    add_library(avalonmm_driver src/avalonmm_driver.cpp)
    # target_include_directories(avalonmm_driver PUBLIC include)
    target_include_directories(avalonmm_driver PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
    
    # install headers
    install(DIRECTORY include/
      DESTINATION include/${PROJECT_NAME}
    )
    

    # accelerated_doublevadd_publisher_agilex
    add_executable(accelerated_doublevadd_publisher_agilex
      src/accelerated_doublevadd_publisher_agilex.cpp
    )
    target_include_directories(accelerated_doublevadd_publisher_agilex PUBLIC include)
    target_link_libraries(accelerated_doublevadd_publisher_agilex
      avalonmm_driver
      pthread
    )
    ament_target_dependencies(accelerated_doublevadd_publisher_agilex
      rclcpp
      std_msgs
      tracetools 
      tracetools_acceleration
    )

    # export
    ament_export_include_directories(include)
    ament_export_libraries(avalonmm_driver)
    ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
    install(TARGETS 
      avalonmm_driver
      EXPORT export_${PROJECT_NAME}
      LIBRARY DESTINATION lib
      ARCHIVE DESTINATION lib
      RUNTIME DESTINATION bin
      INCLUDES DESTINATION include
    )        

    # # sadd kernel
    # oneapi_acceleration_kernel(
    #   NAME add
    #   FILE src/sadd.cpp
    #   FPGA_EMIF_ENABLED 0
    #   FPGA_EMIF_CTRL_BASE_ADDRESS 0x4000
    #   FPGA_EMIF_HOST_ADDR_WIDTH 31
    #   FPGA_EMIF_AGENT_ADDR_WIDTH 27
    #   AVMM_HOST auto
    #   INCLUDE
    #     include
    #   TYPE
    #     fpga
    #   USER_SUBSYSTEM src/sed_sadd_user_subsystem.tcl
    # )

    # vadd kernel
    oneapi_acceleration_kernel(
      NAME accelerated_doublevadd
      FILE src/doublevadd_oneapi.cpp
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
        accelerated_doublevadd_publisher_agilex
      DESTINATION 
        lib/${PROJECT_NAME}
    )

  endif()  # ROS_ONEAPI
endif()  # ROS_ACCERATION

ament_package()
