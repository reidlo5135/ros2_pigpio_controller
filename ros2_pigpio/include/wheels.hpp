#ifndef WHEELS
#define WHEELS

#include <chrono>
#include <memory>
#include <sstream>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "HBBridge.hpp"
#include <pigpiod_if2.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

class WheelsWriter : public rclcpp::Node {
	private :
		int pi_;
	    	int pin_;
	    	HBridge right_;
	    	HBridge left_;
		std::shared_ptr<rclcpp::Node> ros_node_ptr_;
	    	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ros_cmd_vel_subscription_ptr_;
		void ros_cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr twist_callback_ptr) const;
		void set_pin(int pin_ain1, int pin_ain2, int pin_apwm, int pin_bin1, int pin_bin2, int pin_bpwm);
		void set_pin(int pin_m1a, int pin_m1b, int pin_m2a, int pin_m2b);
	public :
		WheelsWriter();
		virtual ~WheelsWriter();
};


#endif
