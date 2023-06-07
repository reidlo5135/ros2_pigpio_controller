#ifndef SERVO_CONTROLLER
#define SERVO_CONTROLLER

#include <signal.h>
#include <pigpiod_if2.h>
#include "hb_bridge.h"
#include "rclc/rclc.h"
#include "rclc/executor.h"
#include "geometry_msgs/msg/twist.h"
#include "rcutils/logging_macros.h"

#define RCL_NODE_NAME "rclc_servo_controller"
#define RCL_CMD_VEL_SUBSCRIPTION_TOPIC "/cmd_vel"

// motor left
extern int pin_in1a;
extern int pin_in1b;
extern int pin_pwm1;

// motor right
extern int pin_in2a;
extern int pin_in2b;
extern int pin_pwm2;

void sig_handler(int sig);
void rcl_cmd_vel_callback(const void * rclc_cmd_vel_in);
void set_pin(int pin_m1a, int pin_m1b, int pin_m2a, int pin_m2b);
void set_pin_with_pwm(int pin_ain1, int pin_ain2, int pin_apwm, int pin_bin1, int pin_bin2, int pin_bpwm);

#endif