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
target_compile_definitions(harris
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
rclcpp_components_register_nodes(harris "image_proc::HarrisNode")
set(node_plugins "${node_plugins}image_proc::HarrisNode;$<TARGET_FILE:harris>\n")

# Node built with a "components"
add_executable(perception_3nodes src/perception_3nodes.cpp)
target_link_libraries(perception_3nodes harris)
ament_target_dependencies(perception_3nodes rclcpp class_loader image_proc)

# install targets
set (install_targets
  perception_3nodes
)
install(
  TARGETS ${install_targets}
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE launch)
