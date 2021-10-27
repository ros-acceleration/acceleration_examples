cmake_minimum_required(VERSION 3.5)
project(simple_adder)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unknown-pragmas)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_vitis)

################
# simple_adder1
################
if(ROS_ACCELERATION)
  # C simulation and synthesis
  vitis_hls_generate_tcl(
    PROJECT
      project_simpleadder1
    SRC
      src/adder1.cpp
    HEADERS
      include
    TESTBENCH
      src/testbench1.cpp
    TOPFUNCTION
      simple_adder
    CLOCK
      4
    SYNTHESIS
  )

  # #---------------------
  # # NOTE: uncomment if
  # # hw kernel is desired
  # #----------------------
  #
  # # kernel
  # vitis_acceleration_kernel(
  #   NAME simple_adder
  #   FILE src/adder1.cpp
  #   CONFIG src/kv260.cfg
  #   INCLUDE
  #     include
  #   TYPE
  #     # sw_emu
  #     # hw_emu
  #     hw
  #   PACKAGE
  # )
endif()

# cross-compilation
add_executable(simple_adder1 src/testbench1.cpp src/adder1.cpp)
target_include_directories(simple_adder1 PUBLIC include)
# ament_target_dependencies(simple_adder1 rclcpp std_msgs)

install(TARGETS
  simple_adder1
  DESTINATION lib/${PROJECT_NAME}
)


################
# simple_adder2
################

if(ROS_ACCELERATION)
  # C simulation and synthesis
  vitis_hls_generate_tcl(
    PROJECT
      project_simpleadder2
    SRC
      src/adder2.cpp
    HEADERS
      include
    TESTBENCH
      src/testbench2.cpp
    TOPFUNCTION
      simple_adder
    CLOCK
      4 10
    SYNTHESIS
  )

  # #---------------------
  # # NOTE: uncomment if
  # # hw kernel is desired
  # #----------------------
  #
  # # kernel
  # vitis_acceleration_kernel(
  #   NAME simple_adder
  #   FILE src/adder2.cpp
  #   CONFIG src/kv260.cfg
  #   INCLUDE
  #     include
  #   TYPE
  #     # sw_emu
  #     # hw_emu
  #     hw
  #   PACKAGE
  # )
endif()

# cross-compilation
add_executable(simple_adder2 src/testbench2.cpp src/adder2.cpp)
target_include_directories(simple_adder2 PUBLIC include)


install(TARGETS
  simple_adder2
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
