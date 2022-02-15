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
find_package(ament_vitis)
find_package(image_proc REQUIRED)
find_package(vitis_common REQUIRED)

include_directories(include)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

# LTTng flame chart and graphs
# see https://archive.eclipse.org/tracecompass/doc/stable/org.eclipse.tracecompass.doc.user/LTTng-UST-Analyses.html#Flame_Chart_View
set(CMAKE_CXX_FLAGS "-g -O2 -finstrument-functions")

# Node built with a "component"
add_executable(rectify_node src/rectify_node.cpp)
ament_target_dependencies(rectify_node rclcpp class_loader vitis_common image_proc)

# targets
install(TARGETS
  rectify_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
