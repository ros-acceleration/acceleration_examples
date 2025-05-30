cmake_minimum_required(VERSION 3.5)
project(vitis_accelerated_stereolbm)

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
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(Threads REQUIRED)
find_package(ament_vitis)

# add_executable(accelerated_vadd_new
#   src/host_new.cpp)
# target_include_directories(accelerated_vadd_new PUBLIC include)
# target_link_libraries(accelerated_vadd_new
#   ${OpenCL_LIBRARY}
#   pthread)

  find_package(vitis_common REQUIRED)
  find_package(OpenCL REQUIRED)

  # accelerated_vadd
  add_executable(vitis_accelerated_stereolbm
    src/accelerated_node.cpp
    src/main.cpp
  )
  target_include_directories(vitis_accelerated_stereolbm PUBLIC 
	   include
	   $ENV{XILINX_HLS}/common/technology/autopilot
    	   $ENV{XILINX_HLS}/include
	   )

  target_link_libraries(vitis_accelerated_stereolbm
    ${OpenCL_LIBRARY}
    pthread
  )
  ament_target_dependencies(vitis_accelerated_stereolbm
	  rclcpp
	  rclcpp_components 
	  std_msgs 
	  cv_bridge 
	  image_transport 
	  vitis_common  )

if (ROS_VITIS)
  # resize kernel
  vitis_acceleration_kernel(
    NAME stereolbm_accel
    FILE src/xf_stereolbm_accel.cpp
    CONFIG cfg/kr260.cfg
    INCLUDE
      include
      ${CMAKE_INSTALL_PREFIX}/include
      ${CMAKE_INSTALL_PREFIX}/include/vitis_common
    TYPE
      # sw_emu
      # hw_emu
      hw
    LINK
    PACKAGE
    )

endif()  # ROS_VITIS

  install(TARGETS
    vitis_accelerated_stereolbm
    # accelerated_vadd_new
    DESTINATION lib/${PROJECT_NAME}
  )

  install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/data/left.png
    ${CMAKE_CURRENT_SOURCE_DIR}/data/right.png
    DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
