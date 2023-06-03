#include "gpio_pwm_cmd_vel_controller.hpp"

PWMWriter::PWMWriter()
: Node(ROS_NODE_NAME) {
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

       	if (motor_left_pi_ >= 0 && motor_right_pi_ >= 0) {
	       	set_mode(motor_left_pi_, motor_left_pin_, PI_OUTPUT);
		set_mode(motor_right_pi_, motor_right_pin_, PI_OUTPUT);
		RCLCPP_INFO(ros_node_ptr_->get_logger(), "create subscription int16");
		std::stringstream ss;
     		ss << "gpio_pwm_" << std::setw(2) << motor_left_pin_;
		ros_int16_subscription_ptr_ = ros_node_ptr_->create_subscription<std_msgs::msg::Int16>(
				 "/int16",
				 10,
				 [this](const std_msgs::msg::Int16::SharedPtr call) {
				 	int16_t pwm_dutycycle = call->data;

 				        set_PWM_dutycycle(motor_left_pi_, motor_left_pin_, pwm_dutycycle);
				        RCLCPP_INFO(ros_node_ptr_->get_logger(), "[LEFT MOTOR] Write  %03d on GPIO-%d", pwm_dutycycle, motor_left_pin_);

				        set_PWM_dutycycle(motor_right_pi_, motor_right_pin_, pwm_dutycycle);
				        RCLCPP_INFO(ros_node_ptr_->get_logger(), "[RIGHT MOTOR] Write  %03d on GPIO-%d", pwm_dutycycle, motor_right_pin_);

				 }
		 );
		ros_cmd_vel_subscription_ptr_ = ros_node_ptr_->create_subscription<geometry_msgs::msg::Twist>(
				ROS_CMD_VEL_SUBSCRIPTION_TOPIC,
				rclcpp::QoS(rclcpp::KeepLast(ROS_DEFAULT_QOS)),
				std::bind(&PWMWriter::ros_cmd_vel_subscription_callback, this, _1)
		);
       	} else {
		RCLCPP_ERROR(ros_node_ptr_->get_logger(), "cannot connect pigpiod");
	       	rclcpp::shutdown();
	       	exit(1);
       	}
}

PWMWriter::~PWMWriter() {
    set_mode(motor_left_pi_, motor_left_pin_, PI_INPUT);
    pigpio_stop(motor_left_pi_);

    set_mode(motor_right_pi_, motor_right_pin_, PI_INPUT);
    pigpio_stop(motor_right_pi_);
}

void PWMWriter::ros_cmd_vel_subscription_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_callback_msg_ptr) const {
	int16_t pwm_dutycycle = 250;
	int16_t accel = (int)(pwm_dutycycle * cmd_vel_callback_msg_ptr->linear.x);

	set_PWM_dutycycle(motor_left_pi_, motor_left_pin_, accel);
        RCLCPP_INFO(ros_node_ptr_->get_logger(), "[LEFT MOTOR] Write  %03d on GPIO-%d", pwm_dutycycle, motor_left_pin_);
        
        set_PWM_dutycycle(motor_right_pi_, motor_right_pin_, accel);
	RCLCPP_INFO(ros_node_ptr_->get_logger(), "[RIGHT MOTOR] Write  %03d on GPIO-%d", pwm_dutycycle, motor_right_pin_);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PWMWriter>());
  rclcpp::shutdown();
  return 0;
}
