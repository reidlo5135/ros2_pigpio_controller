#include "gpio_reader.hpp"

DigitalReader::DigitalReader()
: Node("gpio_publisher"),
ros_timer_count_(0) {
	ros_node_ptr_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});

	ros_node_ptr_->declare_parameter<int>("motor_left_pin", 18);
        ros_node_ptr_->get_parameter("motor_left_pin", motor_left_pin_);
        RCLCPP_INFO(ros_node_ptr_->get_logger(), "[LEFT MOTOR] Write GPIO-%02d", motor_left_pin_);
        motor_left_pi_ = pigpio_start(NULL, NULL);
        RCLCPP_INFO(ros_node_ptr_->get_logger(), "[LEFT MOTOR] GPIO pi : %d, GPIO pin : %d", motor_left_pi_, motor_left_pin_);

        ros_node_ptr_->declare_parameter<int>("motor_right_pin", 12);
        ros_node_ptr_->get_parameter("motor_right_pin", motor_right_pin_);
        RCLCPP_INFO(ros_node_ptr_->get_logger(), "[RIGHT MOTOR] Write GPIO-%02d", motor_right_pin_);
        motor_right_pi_ = pigpio_start(NULL, NULL);
        RCLCPP_INFO(ros_node_ptr_->get_logger(), "[RIGHT MOTOR] GPIO pi : %d, GPIO pin : %d", motor_right_pi_, motor_right_pin_);

   	ros_node_ptr_->declare_parameter<bool>("is_motor_left_pull_up", true);
	ros_node_ptr_->declare_parameter<bool>("is_motor_right_pull_up", true);
    	ros_node_ptr_->get_parameter("is_motor_left_pull_up", is_motor_left_pull_up_);
	ros_node_ptr_->get_parameter("is_motor_right_pull_up", is_motor_right_pull_up_);

	bool is_motor_ready = is_motor_left_pull_up_ && is_motor_right_pull_up_;

    	if (is_motor_ready) {
	      RCLCPP_INFO(ros_node_ptr_->get_logger(), "Read GPIO-%02d (PULL_UP)", motor_left_pin_);
	      RCLCPP_INFO(ros_node_ptr_->get_logger(), "Read GPIO-%02d (PULL_UP)", motor_right_pin_);
       	} else {
	      RCLCPP_INFO(ros_node_ptr_->get_logger(), "Read GPIO-%02d (PULL_DOWN)", motor_left_pin_);
	      RCLCPP_INFO(ros_node_ptr_->get_logger(), "Read GPIO-%02d (PULL_DOWN)", motor_right_pin_);
       	}

    	if (motor_left_pi_ < 0 || motor_right_pi_ < 0) {
	      RCLCPP_ERROR(ros_node_ptr_->get_logger(), "cannot connect pigpiod");
	      rclcpp::shutdown();
	      exit(1);
    	} else {
	      set_mode(motor_left_pi_, motor_left_pin_, PI_INPUT);
	      set_mode(motor_right_pi_, motor_right_pin_, PI_INPUT);
	      if (is_motor_ready) {
		      set_pull_up_down(motor_left_pi_, motor_left_pin_, PI_PUD_UP);
		      set_pull_up_down(motor_right_pi_, motor_right_pin_, PI_PUD_UP);
	      } else {
		      set_pull_up_down(motor_left_pi_, motor_left_pin_, PI_PUD_DOWN);
		      set_pull_up_down(motor_right_pi_, motor_right_pin_, PI_PUD_DOWN);
	      }
	      int motor_pin = motor_left_pin_ + motor_right_pin_;
	      std::stringstream ss;
	      ss << "gpio_input_" << std::setw(2) << motor_pin;
	      ros_motor_pull_up_publisher_ptr_ = ros_node_ptr_->create_publisher<std_msgs::msg::Bool>(ss.str(), 10);
	      ros_timer_ptr_ = ros_node_ptr_->create_wall_timer(
	          std::chrono::milliseconds(500), std::bind(&DigitalReader::ros_timer_callback, this));
    	}
}

DigitalReader::~DigitalReader() {
	set_mode(motor_left_pi_, motor_left_pin_, PI_INPUT);
    	pigpio_stop(motor_left_pi_);

	set_mode(motor_right_pi_, motor_right_pin_, PI_INPUT);
    	pigpio_stop(motor_right_pi_);
}

void DigitalReader::ros_timer_callback() {
	auto message = std_msgs::msg::Bool();
    	message.data = gpio_read(motor_left_pi_, motor_left_pin_); 
    	RCLCPP_INFO(ros_node_ptr_->get_logger(), "Publishing: '%d'", message.data);
    	ros_motor_pull_up_publisher_ptr_->publish(message);
}	

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DigitalReader>());
  rclcpp::shutdown();

  return 0;
}
