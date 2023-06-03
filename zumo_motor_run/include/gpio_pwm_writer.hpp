#ifndef GPIO_PWM_WRTIER
#define GPIO_PWM_WRITER

#include <chrono>
#include <memory>
#include <thread>
#include <sstream>
#include <iomanip>

#include <pigpiod_if2.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class PWMWriter : public rclcpp::Node {
	private:
	  int motor_left_pi_;
	  int motor_left_pin_;
	  int motor_right_pi_;
	  int motor_right_pin_;
	  std::shared_ptr<rclcpp::Node> ros_node_ptr_;
	  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr ros_std_msgs_int16_subscription_ptr_;
	  void int16_subscription_callback(const std_msgs::msg::Int16::SharedPtr int16_callback_msg_ptr) const;
	public:
	  PWMWriter();
	  virtual ~PWMWriter();
};

#endif
