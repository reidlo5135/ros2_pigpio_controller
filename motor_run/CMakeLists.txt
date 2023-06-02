cmake_minimum_required(VERSION 3.5)
project(motor_run)

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
find_library(WIRINGPI_LIB wiringPi)

add_executable(
  motor_boot
  src/motor_boot.cpp
)

ament_target_dependencies(
  motor_boot
  rclcpp
)

target_link_libraries(
  motor_boot
  ${WIRINGPI_LIB}
)

install(TARGETS
  motor_boot
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
