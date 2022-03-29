cmake_minimum_required(VERSION 3.5)
project(perception_3nodes)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unknown-pragmas)
endif()

find_package(ament_vitis)
find_package(ament_cuda)  # enable CUDA capabilities, if available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if (DEFINED ROS_VITIS)
  find_package(vitis_common)
  find_package(OpenCL REQUIRED)  
endif()

include_directories(include)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

# LTTng flame chart and graphs
# see https://archive.eclipse.org/tracecompass/doc/stable/org.eclipse.tracecompass.doc.user/LTTng-UST-Analyses.html#Flame_Chart_View
set(CMAKE_CXX_FLAGS "-g -O2 -finstrument-functions")


# harris component
ament_auto_add_library(harris SHARED
  src/harris.cpp)
ament_target_dependencies(harris tracetools_image_pipeline)
target_include_directories(harris PUBLIC
  # $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include/vitis_common>
  $ENV{XILINX_HLS}/common/technology/autopilot
  $ENV{XILINX_HLS}/include)
target_compile_definitions(harris
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
rclcpp_components_register_nodes(harris "image_proc::HarrisNode")
set(node_plugins "${node_plugins}image_proc::HarrisNode;$<TARGET_FILE:harris>\n")

if (DEFINED ROS_VITIS)

  # 3nodes_harris_fpga component (with OpenCL)
  ament_auto_add_library(3nodes_harris_fpga SHARED src/harris_fpga.cpp)
  target_include_directories(3nodes_harris_fpga PUBLIC
    # $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include/vitis_common>
    $ENV{XILINX_HLS}/common/technology/autopilot
    $ENV{XILINX_HLS}/include)
  target_link_libraries(3nodes_harris_fpga ${OpenCL_LIBRARY})
  target_compile_definitions(3nodes_harris_fpga PRIVATE "COMPOSITION_BUILDING_DLL")
  rclcpp_components_register_nodes(3nodes_harris_fpga "perception_3nodes::HarrisNodeFPGA")
  set(node_plugins "${node_plugins}perception_3nodes::HarrisNodeFPGA;$<TARGET_FILE:harris_fpga>\n")

  # 3nodes_rectify_fpga component (with OpenCL)
  ament_auto_add_library(3nodes_rectify_fpga SHARED src/rectify_fpga.cpp)
  target_include_directories(3nodes_rectify_fpga PUBLIC
    # $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include/vitis_common>
    $ENV{XILINX_HLS}/common/technology/autopilot
    $ENV{XILINX_HLS}/include
  )
  target_link_libraries(3nodes_rectify_fpga ${OpenCL_LIBRARY})
  ament_target_dependencies(3nodes_rectify_fpga vitis_common)
  target_compile_definitions(3nodes_rectify_fpga PRIVATE "COMPOSITION_BUILDING_DLL")
  rclcpp_components_register_nodes(3nodes_rectify_fpga "perception_3nodes::RectifyNodeFPGAStreamlined")
  set(node_plugins "${node_plugins}perception_3nodes::RectifyNodeFPGAStreamlined;$<TARGET_FILE:rectify_fpga>\n")

  # 3nodes_resize_fpga component (with OpenCL)
  ament_auto_add_library(3nodes_resize_fpga SHARED src/resize_fpga.cpp)
  target_include_directories(3nodes_resize_fpga PUBLIC
    # $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include/vitis_common>
    $ENV{XILINX_HLS}/common/technology/autopilot
    $ENV{XILINX_HLS}/include
  )
  target_link_libraries(3nodes_resize_fpga ${OpenCL_LIBRARY})
  ament_target_dependencies(3nodes_resize_fpga vitis_common)
  target_compile_definitions(3nodes_resize_fpga PRIVATE "COMPOSITION_BUILDING_DLL")
  rclcpp_components_register_nodes(3nodes_resize_fpga "perception_3nodes::ResizeNodeFPGAStreamlined")
  set(node_plugins "${node_plugins}perception_3nodes::ResizeNodeFPGAStreamlined;$<TARGET_FILE:resize_fpga>\n")


  #####################
  # Synth
  #####################
  # # rectify_accel_streamlined kernel
  # vitis_acceleration_kernel(
  #   NAME rectify_accel_streamlined
  #   FILE src/kernels/xf_rectify_accel.cpp
  #   CONFIG cfg/kv260_perception_3nodes.cfg
  #   # CLOCK 250000000:rectify_accel_streamlined
  #   INCLUDE
  #     include/image_proc
  #     ${CMAKE_INSTALL_PREFIX}/include
  #   TYPE hw    
  # )

  # # resize_accel_streamlined kernel
  # vitis_acceleration_kernel(
  #   NAME resize_accel_streamlined
  #   FILE src/kernels/xf_resize_accel.cpp
  #   CONFIG cfg/kv260_perception_3nodes.cfg
  #   INCLUDE 
  #     include/image_proc 
  #     ${CMAKE_INSTALL_PREFIX}/include
  #   TYPE hw
  # )

  # cornerHarris_accel kernel
  vitis_acceleration_kernel(
    NAME cornerHarris_accel
    FILE src/kernels/xf_harris_accel.cpp
    CONFIG cfg/kv260_perception_3nodes.cfg
    INCLUDE
      include/image_proc
      ${CMAKE_INSTALL_PREFIX}/include
    TYPE hw 
  )  

  #####################
  # Place and route
  #####################
  # image_proc_streamlined kernel
  vitis_link_kernel(
    OUTPUT perception_3nodes
    KERNELS resize_accel_streamlined rectify_accel_streamlined cornerHarris_accel
    CONFIG cfg/kv260_perception_3nodes.cfg
  )

  add_executable(perception_3nodes_fpga src/perception_3nodes_fpga.cpp)
  target_link_libraries(perception_3nodes_fpga 3nodes_harris_fpga 3nodes_resize_fpga 3nodes_rectify_fpga)
  ament_target_dependencies(perception_3nodes_fpga rclcpp class_loader)

endif()  # ROS_VITIS

# Node built with a "components"
add_executable(perception_3nodes src/perception_3nodes.cpp)
target_link_libraries(perception_3nodes harris)
ament_target_dependencies(perception_3nodes rclcpp class_loader image_proc)

# install targets
set (install_targets
  perception_3nodes
)
if (DEFINED ROS_VITIS)
  set (install_targets ${install_targets} 
    perception_3nodes_fpga 
  )
endif()  # ROS_VITIS
install(
  TARGETS ${install_targets}
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE launch)
