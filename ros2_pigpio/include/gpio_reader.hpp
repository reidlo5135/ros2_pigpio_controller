#ifndef GPIO_READER
#define GPIO_READER

#include <chrono>
#include <memory>
#include <thread>
#include <sstream>
#include <iomanip>

#include <pigpiod_if2.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class DigitalReader : public rclcpp::Node {
	private :
		 int motor_left_pi_;
	 	 int motor_left_pin_;
		 int motor_right_pi_;
		 int motor_right_pin_;
       		 std::shared_ptr<rclcpp::Node> ros_node_ptr_;
       		 bool is_motor_left_pull_up_;
		 bool is_motor_right_pull_up_;
       		 rclcpp::TimerBase::SharedPtr ros_timer_ptr_;
       		 rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ros_motor_pull_up_publisher_ptr_;
       		 size_t ros_timer_count_;
		 void ros_timer_callback();
	 public :
		 DigitalReader();
		 virtual ~DigitalReader();

};


#endif
