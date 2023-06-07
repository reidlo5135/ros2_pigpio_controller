#include "servo_controller/servo_controller.h"

void sig_handler(int sig) {
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "manual move stopped..\n");
	signal(sig, SIG_IGN);
	exit(0);
}

void rcl_cmd_vel_callback(const void * rcl_cmd_vel_in) {
    const geometry_msgs__msg__Twist * rcl_cmd_vel_msgs = (const geometry_msgs__msg__Twist *)rcl_cmd_vel_in;
    static int pwm_r, pwm_l;

    pwm_r = 255 * (rcl_cmd_vel_msgs->linear.x + rcl_cmd_vel_msgs->angular.z);
    pwm_l = 255 * (rcl_cmd_vel_msgs->linear.x - rcl_cmd_vel_msgs->angular.z);

    if(abs(pwm_r) > 255) {
        float c = 255.0 / abs(pwm_r);
        pwm_r *= c;
        pwm_l *= c;
    }

    if(abs(pwm_l) > 255) {
        float c = 255.0 / abs(pwm_l);
        pwm_r *= c;
        pwm_l *= c;
    }

    hb_drive(pwm_r);
    hb_drive(pwm_l);

    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "write motor power R : %03d, L : %03d", pwm_r, pwm_l);
}

void set_pin(int pin_m1a, int pin_m1b, int pin_m2a, int pin_m2b) {
    hb_set_pin(pi_, pin_m1a, pin_m1b);
    hb_set_pin(pi_, pin_m2a, pin_m2b);
}

void set_pin_with_pwm(int pin_ain1, int pin_ain2, int pin_apwm, int pin_bin1, int pin_bin2, int pin_bpwm) {
    hb_set_pin_pwm(pi_, pin_ain1, pin_ain2, pin_apwm);
    hb_set_pin_pwm(pi_, pin_bin1, pin_bin2, pin_bpwm);
}

int main(int argc, const char * const * argv) {
    rcl_allocator_t rcl_allocator = rcl_get_default_allocator();
    rclc_support_t rclc_support;
    rcl_ret_t rc;
	RCUTILS_LOGGING_AUTOINIT;
	rcutils_logging_set_logger_level(RCL_NODE_NAME, RCUTILS_LOG_SEVERITY_INFO);

    rc = rclc_support_init(&rclc_support, argc, argv, &rcl_allocator);

    if (rc != RCL_RET_OK) {
		RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "Failed to initialize rclc\n");
        return EXIT_FAILURE;
    } else {
		RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "Succeeded to initialize rclc with RC [%i]\n", rc);
    }

    rcl_node_t rcl_node = rcl_get_zero_initialized_node();
    rc = rclc_node_init_default(&rcl_node, RCL_NODE_NAME, "", &rclc_support);

    if (rc != RCL_RET_OK) {
		RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "Failed to initialized rclc_node\n");
        return EXIT_FAILURE;
    } else {
		RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "Succeeded to initialize rclc_node with RC [%i]\n", rc);
    }

    signal(SIGINT,  &sig_handler);
	signal(SIGTSTP, &sig_handler);

    rcl_subscription_t rcl_cmd_vel_subscription = rcl_get_zero_initialized_subscription();
    const char * rcl_cmd_vel_topic = RCL_CMD_VEL_SUBSCRIPTION_TOPIC;
    const rosidl_message_type_support_t * rcl_twist_message_type = ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist);

    if (rc != RCL_RET_OK) {
		RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "Failed to create %s subscription, error code RC [%i]\n", rcl_cmd_vel_topic, rc);
        return EXIT_FAILURE;
    } else {
		RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "Succeeded to create %s subscription, success code RC [%i]\n", rcl_cmd_vel_topic, rc);
    }

    geometry_msgs__msg__Twist rcl_twist_msgs;
	geometry_msgs__msg__Twist__init(&rcl_twist_msgs);

    rclc_executor_t rclc_executor;
	rclc_executor = rclc_executor_get_zero_initialized_executor();

    unsigned int num_handles = 1;
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "number of DDS handles: %u\n", num_handles);
    rc = rclc_executor_init(&rclc_executor, &rclc_support.context, num_handles, &rcl_allocator);

    if (rc != RCL_RET_OK) {
		RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "Failed to initialize rclc executor, error code RC [%i]\n", rc);
        return EXIT_FAILURE;
    } else {
		RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "Succeeded to initialize rclc executor, success code RC [%i]\n", rc);
    }

    rc = rclc_executor_add_subscription(&rclc_executor, &rcl_cmd_vel_subscription, &rcl_twist_msgs, &rcl_cmd_vel_callback, ON_NEW_DATA);

    if(rc != RCL_RET_OK) {
        RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "Failed to add %s subscription into rcl executor, error code RC [%i]\n", rcl_cmd_vel_topic, rc);
    } else {
        RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "Succeeded to add %s subscription into rcl executor, success code RC [%i]\n", rcl_cmd_vel_topic, rc);
    }

    rc = rclc_executor_spin(&rclc_executor);

    if (rc != RCL_RET_OK) {
		RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "Failed to spin rclc executor, error code RC [%i]\n", rc);
        return EXIT_FAILURE;
    } else {
		RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "Succeeded to spin rclc executor, success code RC [%i]\n", rc);
    }

    rc = rclc_executor_fini(&rclc_executor);
    rc += rcl_subscription_fini(&rcl_cmd_vel_subscription, &rcl_node);
    geometry_msgs__msg__Twist__fini(&rcl_twist_msgs);

    if (rc != RCL_RET_OK) {
        RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "Error while cleaning up!\n");
        return EXIT_FAILURE;
    } else {
		RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "Succeeded to cleaning up rcl\n");
	}

    return rc;
}