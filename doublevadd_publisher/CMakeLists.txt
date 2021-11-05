cmake_minimum_required(VERSION 3.5)
project(doublevadd_publisher)

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
find_package(std_msgs REQUIRED)
find_package(ament_vitis)
find_package(tracetools REQUIRED)
find_package(tracetools_acceleration REQUIRED)

include_directories(include)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

# Traditional Node with publisher
add_executable(doublevadd_publisher src/doublevadd_publisher.cpp src/vadd.cpp)
target_include_directories(doublevadd_publisher PUBLIC include)
ament_target_dependencies(doublevadd_publisher rclcpp std_msgs tracetools tracetools_acceleration)

set(COMPONENT_AMENT_DEPENDENCIES
  rclcpp
  rclcpp_components
  std_msgs
  tracetools
  tracetools_acceleration)

# Components
## Simple composition
add_library(doublevadd_component SHARED
src/doublevadd_component.cpp src/vadd.cpp)
target_compile_definitions(doublevadd_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(doublevadd_component ${COMPONENT_AMENT_DEPENDENCIES})
rclcpp_components_register_nodes(doublevadd_component "composition::DoubleVaddNode")
set(node_plugins "${node_plugins}composition::DoubleVaddNode;$<TARGET_FILE:doublevadd_component>\n")

## Using type adaptation
##  only available in ROS_DISTRO="rolling" for now
if($ENV{ROS_DISTRO} STREQUAL "rolling")
  add_library(doublevadd_component_typeadapter SHARED
  src/doublevadd_component_typeadapter.cpp src/vadd.cpp)
  target_compile_definitions(doublevadd_component_typeadapter
    PRIVATE "COMPOSITION_BUILDING_DLL")
  ament_target_dependencies(doublevadd_component_typeadapter ${COMPONENT_AMENT_DEPENDENCIES})
  rclcpp_components_register_nodes(doublevadd_component_typeadapter "composition::DoubleVaddTypeAdapter")
  set(node_plugins "${node_plugins}composition::DoubleVaddTypeAdapter;$<TARGET_FILE:doublevadd_component_typeadapter>\n")
endif()

# ## Adaptive Component
# add_library(doublevadd_component_adaptive SHARED
# src/doublevadd_component_adaptive.cpp src/vadd.cpp)
# target_compile_definitions(doublevadd_component_adaptive
#   PRIVATE "COMPOSITION_BUILDING_DLL")
# ament_target_dependencies(doublevadd_component_adaptive ${COMPONENT_AMENT_DEPENDENCIES})
# rclcpp_components_register_nodes(doublevadd_component_adaptive "composition::DoubleVaddAdaptiveComponent")
# set(node_plugins "${node_plugins}composition::DoubleVaddAdaptiveComponent;$<TARGET_FILE:doublevadd_component_adaptive>\n")

# Node built with a "component"
add_executable(doublevadd_node src/doublevadd_node.cpp src/vadd.cpp)
target_link_libraries(doublevadd_node doublevadd_component)
ament_target_dependencies(doublevadd_node rclcpp class_loader)
# If the interfaces are created in the same package that utilizes them,
# you'll need to link against them using the ``rosidl_target_interfaces`` macro.
#
# rosidl_target_interfaces(doublevadd_node
#       ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Adaptive Node built with a "component" defined altogether
if(ROS_ACCELERATION)
  find_package(OpenCL REQUIRED)
  find_package(vitis_common REQUIRED)

  add_executable(doublevadd_node_adaptive src/doublevadd_node_adaptive.cpp src/vadd.cpp)
  target_include_directories(doublevadd_node_adaptive PUBLIC include)
  target_link_libraries(doublevadd_node_adaptive ${OpenCL_LIBRARY})
  ament_target_dependencies(doublevadd_node_adaptive
    rclcpp
    std_msgs
    tracetools
    tracetools_acceleration
    class_loader
    vitis_common)
endif()


if(ROS_ACCELERATION)
  # C simulation and synthesis
  vitis_hls_generate_tcl(
    PROJECT
      project_doublevadd_publisher
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
endif()

# install components
install(TARGETS
  doublevadd_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(ROS_ACCELERATION)
  install(TARGETS
    doublevadd_publisher
    doublevadd_node
    doublevadd_node_adaptive
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  install(TARGETS
    doublevadd_publisher
    doublevadd_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

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
