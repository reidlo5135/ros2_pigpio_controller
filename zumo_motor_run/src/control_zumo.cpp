#include "control_zumo.hpp"

RCLZumoMotorRunner::RCLZumoMotorRunner()
: Node(RCL_NODE_NAME) {
	ros_node_ptr_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
	ros_cmd_vel_subscription_ptr_ = ros_node_ptr_->create_subscription<geometry_msgs::msg::Twist>(
		RCL_CMD_VEL_TOPIC,
		rclcpp::QoS(rclcpp::KeepLast(RCL_DEFAULT_QOS)),
		[this](const geometry_msgs::msg::Twist::SharedPtr callback_twist_ptr) {
			RCLCPP_INFO(ros_node_ptr_->get_logger(), "callback twist linear x : %f", callback_twist_ptr->linear.x);
		}
	);
	RCLCPP_INFO(ros_node_ptr_->get_logger(), "RCLZumoMotorRunner has started...");
}

RCLZumoMotorRunner::~RCLZumoMotorRunner() {

}

void move_forward() {
    std::cout << "forward" << '\n';

    pwmWrite(MOTOR_A1_PIN, DUTY_CYCLE);
    pwmWrite(MOTOR_B1_PIN, DUTY_CYCLE);

    digitalWrite(MOTOR_A2_PIN, LOW);
    digitalWrite(MOTOR_B2_PIN, LOW);
}

void move_backward() {
    std::cout << "back" << '\n';

    pwmWrite(MOTOR_A1_PIN, DUTY_CYCLE);
    pwmWrite(MOTOR_B1_PIN, DUTY_CYCLE);

    digitalWrite(MOTOR_A2_PIN, HIGH);
    digitalWrite(MOTOR_B2_PIN, HIGH);
}

void move_left() {
    std::cout << "left" << '\n';

    pwmWrite(MOTOR_A1_PIN, DUTY_CYCLE);
    pwmWrite(MOTOR_B1_PIN, DUTY_CYCLE);

    digitalWrite(MOTOR_A2_PIN, LOW);
    digitalWrite(MOTOR_B2_PIN, HIGH);
}

void move_right() {
    std::cout << "right" << '\n';

    pwmWrite(MOTOR_A1_PIN, DUTY_CYCLE);
    pwmWrite(MOTOR_B1_PIN, DUTY_CYCLE);

    digitalWrite(MOTOR_A2_PIN, HIGH);
    digitalWrite(MOTOR_B2_PIN, LOW);
}

void stop() {
    std::cout << "stop" << '\n';

    pwmWrite(MOTOR_A1_PIN, PWM_STOP);
    pwmWrite(MOTOR_B1_PIN, PWM_STOP);
}

int main(int argc, char** argv) {
    wiringPiSetup();

    pinMode(MOTOR_A1_PIN, PWM_OUTPUT);
    pinMode(MOTOR_A2_PIN, OUTPUT);
    pinMode(MOTOR_B1_PIN, PWM_OUTPUT);
    pinMode(MOTOR_B2_PIN, OUTPUT);

    pwmSetMode(PWM_MODE_MS);
    pwmSetRange(PWM_RANGE);
    pwmSetClock(PWM_CLOCK);

    try {
        move_forward();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        stop();
        move_backward();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        stop();
        move_left();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        stop();
        move_right();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        stop();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
        std::cout << "Exception occurred" << '\n';
    }

    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<RCLZumoMotorRunner>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    stop();
    pinMode(MOTOR_A1_PIN, INPUT);
    pinMode(MOTOR_B1_PIN, INPUT);

    return 0;
}
