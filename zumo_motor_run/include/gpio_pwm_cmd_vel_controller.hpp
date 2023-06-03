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

#define ROS_NODE_NAME "gpio_cmd_vel_controller"
#define ROS_DEFAULT_QOS 10
#define ROS_CMD_VEL_SUBSCRIPTION_TOPIC "/cmd_vel"

using namespace std::chrono_literals;
using std::placeholders::_1;

class PWMWriter : public rclcpp::Node {
	private:
	  int motor_left_pi_;
	  int motor_left_pin_;
	  int motor_right_pi_;
	  int motor_right_pin_;
	  std::shared_ptr<rclcpp::Node> ros_node_ptr_;
	  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ros_cmd_vel_subscription_ptr_;
	  void ros_cmd_vel_subscription_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_callback_msg_ptr) const;
	public:
	  PWMWriter();
	  virtual ~PWMWriter();
};

#endif
