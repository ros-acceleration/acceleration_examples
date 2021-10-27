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

# Component
add_library(doublevadd_component SHARED
src/doublevadd_component.cpp)
target_compile_definitions(doublevadd_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(doublevadd_component
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "tracetools"
  "tracetools_acceleration")
rclcpp_components_register_nodes(doublevadd_component "composition::DoubleVaddNode")
set(node_plugins "${node_plugins}composition::DoubleVaddNode;$<TARGET_FILE:doublevadd_component>\n")

# Node built with "component"
add_executable(doublevadd_node src/doublevadd_node.cpp src/vadd.cpp)
target_link_libraries(doublevadd_node doublevadd_component)
ament_target_dependencies(doublevadd_node rclcpp class_loader)
# If the interfaces are created in the same package that utilizes them,
# you'll need to link against them using the ``rosidl_target_interfaces`` macro.
#
# rosidl_target_interfaces(doublevadd_node
#       ${PROJECT_NAME} "rosidl_typesupport_cpp")

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

install(TARGETS
  doublevadd_publisher
  doublevadd_node
  DESTINATION lib/${PROJECT_NAME}
)

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
