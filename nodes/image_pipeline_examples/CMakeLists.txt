cmake_minimum_required(VERSION 3.5)
project(image_pipeline_examples)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unknown-pragmas)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ament_vitis QUIET) # enable Vitis capabilities, if available
find_package(ament_cuda QUIET)  # enable CUDA capabilities, if available
find_package(image_proc REQUIRED)

include_directories(include)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

# LTTng flame chart and graphs
# see https://archive.eclipse.org/tracecompass/doc/stable/org.eclipse.tracecompass.doc.user/LTTng-UST-Analyses.html#Flame_Chart_View
set(CMAKE_CXX_FLAGS "-g -O2 -finstrument-functions")

# Node built with a "component"
add_executable(rectify_node src/rectify_node.cpp)
ament_target_dependencies(rectify_node rclcpp class_loader image_proc)

add_executable(rectify_resize_node src/rectify_resize_node.cpp)
ament_target_dependencies(rectify_resize_node rclcpp class_loader image_proc)

if (ROS_CUDA)
  find_package(isaac_ros_image_proc REQUIRED)

  add_executable(rectify_resize_gpu_node src/rectify_resize_gpu_node.cpp)
  # ament_target_dependencies(rectify_resize_gpu_node rclcpp class_loader image_proc)
  ament_target_dependencies(rectify_resize_gpu_node rclcpp class_loader isaac_ros_image_proc)
  target_include_directories(rectify_resize_gpu_node PUBLIC
        # /home/nvidia/nvidia_ws/src/image_common/image_transport/include
        ${CMAKE_SYSROOT}/opt/nvidia/vpi1/include
  )
endif()  # ROS_CUDA

if (ROS_VITIS) 
  find_package(vitis_common REQUIRED)

  add_executable(rectify_resize_fpga_node src/rectify_resize_fpga_node.cpp)
  ament_target_dependencies(rectify_resize_fpga_node rclcpp class_loader vitis_common image_proc)
  target_include_directories(rectify_resize_fpga_node PUBLIC
  )

  add_executable(rectify_resize_fpga_integrated_node src/rectify_resize_fpga_integrated_node.cpp)
  ament_target_dependencies(rectify_resize_fpga_integrated_node rclcpp class_loader vitis_common image_proc)
  target_include_directories(rectify_resize_fpga_integrated_node PUBLIC
    /home/xilinx/XRT/build/Debug/opt/xilinx/xrt/include  # TODO: replace with ROS 2 header-only library
    /usr/include/xrt  # TODO: replace with ROS 2 header-only library
  )


  add_executable(rectify_resize_fpga_streamlined_node src/rectify_resize_fpga_streamlined_node.cpp)
  ament_target_dependencies(rectify_resize_fpga_streamlined_node rclcpp class_loader vitis_common image_proc)
  target_include_directories(rectify_resize_fpga_streamlined_node PUBLIC
    /home/xilinx/XRT/build/Debug/opt/xilinx/xrt/include  # TODO: replace with ROS 2 header-only library
    /usr/include/xrt  # TODO: replace with ROS 2 header-only library
  )

  if (DEFINED ROS_XRT)
    add_executable(rectify_resize_fpga_streamlined_node_xrt src/rectify_resize_fpga_streamlined_node_xrt.cpp)
    ament_target_dependencies(rectify_resize_fpga_streamlined_node_xrt rclcpp class_loader vitis_common image_proc)
    target_include_directories(rectify_resize_fpga_streamlined_node_xrt PUBLIC
      /home/xilinx/XRT/build/Debug/opt/xilinx/xrt/include  # TODO: replace with ROS 2 header-only library
      /usr/include/xrt  # TODO: replace with ROS 2 header-only library
    )
  endif()  # ROS_XRT

  add_executable(rectify_resize_fpga_integrated_streamlined_node src/rectify_resize_fpga_integrated_streamlined_node.cpp)
  ament_target_dependencies(rectify_resize_fpga_integrated_streamlined_node rclcpp class_loader vitis_common image_proc)
  target_include_directories(rectify_resize_fpga_integrated_streamlined_node PUBLIC
    /home/xilinx/XRT/build/Debug/opt/xilinx/xrt/include  # TODO: replace with ROS 2 header-only library
    /usr/include/xrt  # TODO: replace with ROS 2 header-only library
  )
endif()  # ROS_VITIS

# targets
set (install_targets
  rectify_node
  rectify_resize_node
)

if (DEFINED ROS_CUDA)
  set(install_targets ${install_targets} rectify_resize_gpu_node)
endif()  # ROS_CUDA

if (DEFINED ROS_XRT)
  set(install_targets ${install_targets} rectify_resize_fpga_streamlined_node_xrt)
endif()  # ROS_XRT

if (DEFINED ROS_VITIS)
  set (install_targets ${install_targets} 
    rectify_resize_fpga_node 
    rectify_resize_fpga_integrated_node
    rectify_resize_fpga_streamlined_node
    rectify_resize_fpga_integrated_streamlined_node
  )
endif()  # ROS_VITIS

install(
  TARGETS ${install_targets}
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
