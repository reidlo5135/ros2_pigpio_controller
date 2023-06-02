#ifndef CONTROL_ZUMO
#define CONTROL_ZUMO

#include <iostream>
#include <chrono>
#include <thread>
#include <wiringPi.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#define RCL_NODE_NAME "zumo_motor_runner"
#define RCL_DEFAULT_QOS 10
#define RCL_CMD_VEL_TOPIC "/cmd_vel"

constexpr int PWM_FREQUENCY = 50;
constexpr int DUTY_CYCLE = 50;

constexpr int MOTOR_A1_PIN = 18;  // GPIO 18
constexpr int MOTOR_A2_PIN = 5;   // GPIO 5
constexpr int MOTOR_B1_PIN = 12;  // GPIO 12
constexpr int MOTOR_B2_PIN = 6;   // GPIO 6

constexpr int PWM_RANGE = 100;
constexpr int PWM_CLOCK = 384;
constexpr int PWM_STOP = 0;

class RCLZumoMotorRunner : public rclcpp::Node {
	private :
		std::shared_ptr<rclcpp::Node> ros_node_ptr_;
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ros_cmd_vel_subscription_ptr_;
	public :
		RCLZumoMotorRunner();
		virtual ~RCLZumoMotorRunner();	
};

#endif
