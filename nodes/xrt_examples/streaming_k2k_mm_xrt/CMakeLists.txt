cmake_minimum_required(VERSION 3.5)
project(streaming_k2k_mm_xrt)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unknown-pragmas)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(ament_cmake REQUIRED)
find_package(ament_vitis)
find_package(vitis_common REQUIRED)

include_directories(include)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

# LTTng flame chart and graphs
# see https://archive.eclipse.org/tracecompass/doc/stable/org.eclipse.tracecompass.doc.user/LTTng-UST-Analyses.html#Flame_Chart_View
set(CMAKE_CXX_FLAGS "-g -O2 -finstrument-functions")

if(ROS_ACCELERATION)  
  if (DEFINED ROS_VITIS)

    # build and install binary only if cross-compiling
    if (DEFINED ROS_XRT)
      ############ host #############
      # Node built with a "component"
      add_executable(host src/cmdlineparser.cpp src/logger.cpp src/host.cpp)
      # TODO: figure out how to make proper use of XRT CMake integration instead
      #   i.e. use "find_package(XRT REQUIRED)"
      if (DEFINED CMAKE_SYSROOT)
        target_link_options(host PRIVATE "LINKER:-lxrt_coreutil")
      endif()
      ament_target_dependencies(host vitis_common)
      target_include_directories(host PUBLIC
        /home/xilinx/XRT/build/Debug/opt/xilinx/xrt/include  # TODO: replace with ROS 2 header-only library
        /usr/include/xrt  # Within sysroot, TODO: replace with ROS 2 header-only library
      )

      # targets
      install(TARGETS
        host
        DESTINATION lib/${PROJECT_NAME}
      )
    endif()

    ############ acceleration kernels #############
    # krnl_stream_vadd
    vitis_acceleration_kernel(
      NAME krnl_stream_vadd
      FILE src/krnl_stream_vadd.cpp
      CONFIG src/krnl_stream_vadd_vmult.cfg
      INCLUDE
        ${CMAKE_INSTALL_PREFIX}/include
      TYPE hw 
    )  

    # krnl_stream_vmult
    vitis_acceleration_kernel(
      NAME krnl_stream_vmult
      FILE src/krnl_stream_vmult.cpp
      CONFIG src/krnl_stream_vadd_vmult.cfg
      INCLUDE 
        ${CMAKE_INSTALL_PREFIX}/include
      TYPE hw
    )
    
    # image_proc_integrated kernel
    vitis_link_kernel(
      OUTPUT krnl_stream_vadd_vmult  
      KERNELS krnl_stream_vadd krnl_stream_vmult
      CONFIG src/krnl_stream_vadd_vmult.cfg
    )

  endif()
endif()

ament_package()
