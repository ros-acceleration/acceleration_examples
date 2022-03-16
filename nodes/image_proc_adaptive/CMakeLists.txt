cmake_minimum_required(VERSION 3.5)
project(image_proc_adaptive)

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
find_package(image_proc REQUIRED)
find_package(ament_vitis)
# find_package(tracetools REQUIRED)
# find_package(tracetools_acceleration REQUIRED)
find_package(adaptive_component REQUIRED)

if (ROS_VITIS)
  find_package(vitis_common REQUIRED)
  find_package(OpenCL REQUIRED)

  include_directories(include)

  # create ament index resource which references the libraries in the binary dir
  set(node_plugins "")

  set(COMPONENT_AMENT_DEPENDENCIES
    rclcpp
    rclcpp_components
    vitis_common
    # tracetools
    # tracetools_acceleration
  )

  add_library(image_proc_adaptive SHARED src/image_proc_adaptive.cpp)
  target_compile_definitions(image_proc_adaptive PRIVATE "COMPOSITION_BUILDING_DLL")
  target_link_libraries(image_proc_adaptive
    # image_proc::resize
    # image_proc::resize_fpga
    adaptive_component::adaptive_component
    ${OpenCL_LIBRARY}
  )
  ament_target_dependencies(image_proc_adaptive image_proc ${COMPONENT_AMENT_DEPENDENCIES})
  rclcpp_components_register_nodes(image_proc_adaptive "composition::ResizeNodeAdaptive")
  set(node_plugins "${node_plugins}composition::ResizeNodeAdaptive;$<TARGET_FILE:image_proc_adaptive>\n")

  # Adaptive Node using an Adaptive Component (built with adaptive_component)
  # NOTE: stateless
  add_executable(image_proc_node_adaptive src/image_proc_node_adaptive.cpp)
  target_include_directories(image_proc_node_adaptive PUBLIC include)
  target_link_libraries(image_proc_node_adaptive image_proc_adaptive)
  ament_target_dependencies(image_proc_node_adaptive class_loader)

  # install components
  install(
    TARGETS image_proc_adaptive
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

  # install targets
  install(TARGETS
    image_proc_node_adaptive
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

  # install headers
  install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
  )
endif()  # ROS_VITIS

ament_package()
