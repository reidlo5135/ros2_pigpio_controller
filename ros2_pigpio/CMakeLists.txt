cmake_minimum_required(VERSION 3.5)
project(ros2_pigpio)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclc REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
	include
	include/${PROJECT_NAME}	
)

add_executable(
  gpio_reader
  src/gpio_reader.cpp
)

target_link_libraries(
  gpio_reader
  pigpiod_if2
)

ament_target_dependencies(
  gpio_reader
  rclcpp
  std_msgs
)

add_executable(
  gpio_writer
  src/gpio_writer.cpp
)

target_link_libraries(
  gpio_writer
  pigpiod_if2
)

ament_target_dependencies(
  gpio_writer
  rclcpp
  std_msgs
)

add_executable(
  gpio_pwm_writer
  src/gpio_pwm_writer.cpp
)

target_link_libraries(
  gpio_pwm_writer
  pigpiod_if2
)

ament_target_dependencies(
  gpio_pwm_writer
  rclcpp
  std_msgs
)

add_executable(
  wheels
  src/wheels.cpp
)

target_link_libraries(
  wheels
  pigpiod_if2
)

ament_target_dependencies(
  wheels
  rclcpp
  std_msgs
  geometry_msgs
)

install(TARGETS
  gpio_reader
  gpio_writer
  gpio_pwm_writer
  wheels
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
