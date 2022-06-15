cmake_minimum_required(VERSION 3.5)
project(accelerated_vadd)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unknown-pragmas)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenCL REQUIRED)
find_package(Threads REQUIRED)
find_package(vitis_common REQUIRED)
find_package(ament_vitis)

if (ROS_VITIS)
  # accelerated_vadd
  add_executable(accelerated_vadd
    src/host.cpp
  )
  target_include_directories(accelerated_vadd PUBLIC include)
  target_link_libraries(accelerated_vadd
    ${OpenCL_LIBRARY}
    pthread
  )
  ament_target_dependencies(accelerated_vadd
    vitis_common
  )

  # vadd kernel
  vitis_acceleration_kernel(
    NAME vadd
    FILE src/vadd.cpp
    CONFIG src/kv260.cfg
    CLOCK 100000000:vadd
    INCLUDE
      include
    TYPE
      # sw_emu
      # hw_emu
      hw
    LINK
    PACKAGE
    )

  install(TARGETS
    accelerated_vadd
    DESTINATION lib/${PROJECT_NAME}
  )
endif()  # ROS_VITIS


ament_package()
