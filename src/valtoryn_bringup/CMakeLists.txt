cmake_minimum_required(VERSION 3.8)
project(valtoryn_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config maps worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install worlds from root if present or from package
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../worlds")
  install(
    DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../../worlds"
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()
