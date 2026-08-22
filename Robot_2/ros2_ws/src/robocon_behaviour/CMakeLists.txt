cmake_minimum_required(VERSION 3.8)
project(robocon_behaviour)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(robocon_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)


# BehaviorTree XML configs
install(DIRECTORY behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

# C++ Mission Executor Node
add_executable(mission_executor src/mission_executor.cpp)
target_include_directories(mission_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mission_executor
  rclcpp
  behaviortree_cpp_v3
  robocon_interfaces
  std_srvs
  std_msgs
)
install(TARGETS mission_executor
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
