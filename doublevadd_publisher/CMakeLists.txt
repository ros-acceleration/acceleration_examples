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
find_package(adaptive_component REQUIRED)

include_directories(include)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")


#################
#    Components
#################

set(COMPONENT_AMENT_DEPENDENCIES
  rclcpp
  rclcpp_components
  std_msgs
  tracetools
  tracetools_acceleration)

## Simple composition
add_library(doublevadd_component SHARED
src/doublevadd_component.cpp src/vadd.cpp)
target_compile_definitions(doublevadd_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(doublevadd_component ${COMPONENT_AMENT_DEPENDENCIES})
rclcpp_components_register_nodes(doublevadd_component "composition::DoubleVaddComponent")
set(node_plugins "${node_plugins}composition::DoubleVaddComponent;$<TARGET_FILE:doublevadd_component>\n")

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

## FPGA component
if(ROS_ACCELERATION)
  find_package(OpenCL REQUIRED)
  find_package(vitis_common REQUIRED)

  add_library(doublevadd_component_fpga SHARED src/doublevadd_component_fpga.cpp)
  target_compile_definitions(doublevadd_component_fpga PRIVATE "COMPOSITION_BUILDING_DLL")
  target_link_libraries(doublevadd_component_fpga
    ${OpenCL_LIBRARY}
  )
  ament_target_dependencies(doublevadd_component_fpga
    vitis_common
    ${COMPONENT_AMENT_DEPENDENCIES}
  )
  rclcpp_components_register_nodes(doublevadd_component_fpga "composition::DoubleVaddComponentFPGA")
  set(node_plugins "${node_plugins}composition::DoubleVaddComponentFPGA;$<TARGET_FILE:doublevadd_component_fpga>\n")
endif()


#################
#    Nodes
#################

# Traditional Node with publisher
add_executable(doublevadd_publisher src/doublevadd_publisher.cpp src/vadd.cpp)
target_include_directories(doublevadd_publisher PUBLIC include)
ament_target_dependencies(doublevadd_publisher rclcpp std_msgs tracetools tracetools_acceleration)

# Node built with a "component"
add_executable(doublevadd_node src/doublevadd_node.cpp src/vadd.cpp)
target_link_libraries(doublevadd_node doublevadd_component)
ament_target_dependencies(doublevadd_node rclcpp class_loader)
# If the interfaces are created in the same package that utilizes them,
# you'll need to link against them using the ``rosidl_target_interfaces`` macro.
#
# rosidl_target_interfaces(doublevadd_node
#       ${PROJECT_NAME} "rosidl_typesupport_cpp")

if(ROS_ACCELERATION)
  find_package(OpenCL REQUIRED)
  find_package(vitis_common REQUIRED)

  # Adaptive Node integrated
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

    # Adaptive Node using an Adaptive Component
    add_executable(doublevadd_publisher_adaptive src/doublevadd_publisher_adaptive.cpp)
    target_include_directories(doublevadd_publisher_adaptive PUBLIC include)
    target_link_libraries(doublevadd_publisher_adaptive
      doublevadd_component
      doublevadd_component_fpga
      adaptive_component::adaptive_component
    )
    ament_target_dependencies(doublevadd_publisher_adaptive
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

# install targets and components
if(ROS_ACCELERATION)
  install(TARGETS
    doublevadd_publisher
    doublevadd_publisher_adaptive
    doublevadd_node
    doublevadd_node_adaptive
    DESTINATION lib/${PROJECT_NAME}
  )

  # install components
  install(TARGETS
    doublevadd_component
    doublevadd_component_fpga
    # doublevadd_component_typeadapter  # re-enable with Rolling
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

else()
  # targets
  install(TARGETS
    doublevadd_publisher
    doublevadd_node
    DESTINATION lib/${PROJECT_NAME}
  )

  # install components
  install(TARGETS
    doublevadd_component
    # doublevadd_component_typeadapter  # re-enable with Rolling
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

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
