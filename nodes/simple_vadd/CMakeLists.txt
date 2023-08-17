cmake_minimum_required(VERSION 3.5)
project(simple_vadd)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unknown-pragmas)
endif()

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
find_package(ament_vitis)

if (ROS_VITIS)
  find_package(OpenCL REQUIRED)
  # find_package(vitis_common REQUIRED)

  # simple_vadd
  add_executable(simple_vadd
    src/vadd.cpp
  )
  target_include_directories(simple_vadd PUBLIC include)
  target_link_libraries(simple_vadd
    ${OpenCL_LIBRARY}
    pthread
  )
  # ament_target_dependencies(simple_vadd
  #   vitis_common
  # )

  # vadd kernel
  vitis_acceleration_kernel(
    NAME krnl_vadd
    FILE src/krnl_vadd.cpp
    CONFIG src/kv260.cfg
    # CLOCK 100000000:vadd
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
    simple_vadd
    DESTINATION lib/${PROJECT_NAME}
  )
endif()  # ROS_VITIS


ament_package()
