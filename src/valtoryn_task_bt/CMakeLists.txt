cmake_minimum_required(VERSION 3.8)
project(valtoryn_task_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

# BT XML files are installed as data; C++ node bindings are added in Phase 3
# when the distro pin is decided and the trees can be compiled/tested.
install(DIRECTORY bt/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
