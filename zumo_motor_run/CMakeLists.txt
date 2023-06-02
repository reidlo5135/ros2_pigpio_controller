cmake_minimum_required(VERSION 3.5)
project(zumo_motor_run)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
  include/${PROJECT_NAME}
  /usr/include/wiringPi
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_library(WIRINGPI_LIB wiringPi)

add_executable(
  control_zumo
  src/control_zumo.cpp
)

ament_target_dependencies(
  control_zumo
  rclcpp
  geometry_msgs
)

target_link_libraries(
  control_zumo
  ${WIRINGPI_LIB}
)

install(TARGETS
  control_zumo
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
