cmake_minimum_required(VERSION 3.5)
project(accelerated_vadd_publisher)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unknown-pragmas)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCL REQUIRED)

# accelerated_vadd_publisher
add_executable(accelerated_vadd_publisher 
  src/accelerated_vadd_publisher.cpp 
  src/vadd.cpp
)
target_include_directories(accelerated_vadd_publisher PUBLIC include)
target_link_libraries(accelerated_vadd_publisher
  ${OpenCL_LIBRARY}
)
ament_target_dependencies(accelerated_vadd_publisher 
  rclcpp 
  std_msgs
)

install(TARGETS
  accelerated_vadd_publisher
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
