cmake_minimum_required(VERSION 3.8)
project(gate_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O3 -march=native -DNDEBUG)
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_options(-g)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gate_msgs REQUIRED)
find_package(micros_common_type REQUIRED)
find_package(motor_manager REQUIRED)

include_directories(include ${micros_common_type_INCLUDE_DIRS} ${motor_manager_INCLUDE_DIRS})

set(motor_manager_dependencies "rclcpp" "gate_msgs")
add_executable(motor_manager_node src/motor_manager_node.cpp)
ament_target_dependencies(motor_manager_node ${motor_manager_dependencies})
target_link_libraries(motor_manager_node motor_manager_library)
install(TARGETS motor_manager_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
