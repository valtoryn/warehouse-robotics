cmake_minimum_required(VERSION 3.8)
project(valtoryn_task_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(valtoryn_interfaces REQUIRED)

include_directories(include)

# BT Action Node Library
add_library(valtoryn_task_bt_nodes SHARED
  src/nav2_client_node.cpp
)
ament_target_dependencies(valtoryn_task_bt_nodes
  rclcpp
  rclcpp_action
  behaviortree_cpp
  nav2_msgs
  geometry_msgs
  valtoryn_interfaces
)

# Task Executor Node
add_executable(valtoryn_task_executor
  src/task_executor_node.cpp
)
target_link_libraries(valtoryn_task_executor
  valtoryn_task_bt_nodes
)
ament_target_dependencies(valtoryn_task_executor
  rclcpp
  rclcpp_action
  behaviortree_cpp
  nav2_msgs
  geometry_msgs
)

install(
  TARGETS valtoryn_task_bt_nodes valtoryn_task_executor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY bt/
  DESTINATION share/${PROJECT_NAME}/bt
)

ament_export_include_directories(include)
ament_export_libraries(valtoryn_task_bt_nodes)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  behaviortree_cpp
  nav2_msgs
  geometry_msgs
)

ament_package()
