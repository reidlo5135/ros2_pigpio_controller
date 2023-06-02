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
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(gpio_pwm_writer src/gpio_pwm_writer.cpp)
target_link_libraries(gpio_pwm_writer pigpiod_if2)
ament_target_dependencies(gpio_pwm_writer rclcpp std_msgs)

install(TARGETS
  gpio_pwm_writer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
