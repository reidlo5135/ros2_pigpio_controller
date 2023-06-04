#include "wheels.hpp"

WheelsWriter::WheelsWriter()
: Node("servo_subscriber") {
	ros_node_ptr_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});

	int pin_in1a, pin_in1b, pin_pwm1; // motor1
        int pin_in2a, pin_in2b, pin_pwm2; // motor2

        // motor1
        ros_node_ptr_->declare_parameter<int>("in1a", 18);
        ros_node_ptr_->get_parameter("in1a", pin_in1a);

        ros_node_ptr_->declare_parameter<int>("in1b", 5);
        ros_node_ptr_->get_parameter("in1b", pin_in1b);

        ros_node_ptr_->declare_parameter<int>("pwm1", -1);
        ros_node_ptr_->get_parameter("pwm1", pin_pwm1);

        // motor2
        ros_node_ptr_->declare_parameter<int>("in2a", 12);
        ros_node_ptr_->get_parameter("in2a", pin_in2a);

        ros_node_ptr_->declare_parameter<int>("in2b", 6);
        ros_node_ptr_->get_parameter("in2b", pin_in2b);

        ros_node_ptr_->declare_parameter<int>("pwm2", -1);
        ros_node_ptr_->get_parameter("pwm2", pin_pwm2);

        pi_ = pigpio_start(NULL, NULL);

	if (pi_ >= 0) {
            if (pin_pwm1 >= 0) {
                this->set_pin(
                    pin_in1a, pin_in1b, pin_pwm1,
                    pin_in2a, pin_in2b, pin_pwm2);
                RCLCPP_INFO(ros_node_ptr_->get_logger(),
                            "Right Motor 1 : IN1A=%02d, IN1B=%02d, PWM1=%02d",
                            pin_in1a, pin_in1b, pin_pwm1);
                RCLCPP_INFO(ros_node_ptr_->get_logger(),
                            "Left Motor 2 : IN2A=%02d, IN2B=%02d, PWM2=%02d",
                            pin_in2a, pin_in2b, pin_pwm2);
            } else {
                this->set_pin(pin_in1a, pin_in1b, pin_in2a, pin_in2b);
                RCLCPP_INFO(ros_node_ptr_->get_logger(),
                            "Right Motor 1 : IN1A=%02d, IN1B=%02d",
                            pin_in1a, pin_in1b, pin_pwm1);
                RCLCPP_INFO(ros_node_ptr_->get_logger(),
                            "Left Motor 2 : IN2A=%02d, IN2B=%02d",
                            pin_in2a, pin_in2b, pin_pwm2);
            }
            ros_cmd_vel_subscription_ptr_ = ros_node_ptr_->create_subscription<geometry_msgs::msg::Twist>(
                "wheels", 10, std::bind(&WheelsWriter::ros_cmd_vel_callback, this, _1));
        } else {
            RCLCPP_ERROR(ros_node_ptr_->get_logger(), "cannot connect pigpiod");
            rclcpp::shutdown();
            exit(1);
        }
}

WheelsWriter::~WheelsWriter() {
	pigpio_stop(pi_);
}

void WheelsWriter::ros_cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr twist_callback_ptr) const {
	static int pwm_r, pwm_l;
        // pwm_r = 255*(msg->linear.x +0.5*wheel_distance_*msg->angular.z);
        // pwm_l = 255*(msg->linear.x -0.5*wheel_distance_*msg->angular.z);
        pwm_r = 255 * (twist_callback_ptr->linear.x + twist_callback_ptr->angular.z);
        pwm_l = 255 * (twist_callback_ptr->linear.x - twist_callback_ptr->angular.z);

        // clip duty ratio
        if (std::abs(pwm_r) > 255) {
            float c = 255.0 / std::abs(pwm_r);
            pwm_r *= c;
            pwm_l *= c;
        }

        if (std::abs(pwm_l) > 255) {
            float c = 255.0 / std::abs(pwm_l);
            pwm_r *= c;
            pwm_l *= c;
        }
        right_.drive(pwm_r);
        left_.drive(pwm_l);
        RCLCPP_INFO(ros_node_ptr_->get_logger(), "Write Motor Power R: %03d, L: %03d", pwm_r, pwm_l);

}
void WheelsWriter::set_pin(int pin_ain1, int pin_ain2, int pin_apwm, int pin_bin1, int pin_bin2, int pin_bpwm)  {
        right_.setPin(pi_, pin_ain1, pin_ain2, pin_apwm);
        left_.setPin(pi_, pin_bin1, pin_bin2, pin_bpwm);
}

void WheelsWriter::set_pin(int pin_m1a, int pin_m1b, int pin_m2a, int pin_m2b) {
        right_.setPin(pi_, pin_m1a, pin_m1b);
        left_.setPin(pi_, pin_m2a, pin_m2b);
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelsWriter>());
    rclcpp::shutdown();
    return 0;
}
