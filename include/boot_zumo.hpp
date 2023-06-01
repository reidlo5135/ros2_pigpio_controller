#ifndef BOOT_ZUMO
#define BOOT_ZUMO

#include <iostream>
#include <chrono>
#include <thread>
#include <wiringPi.h>

constexpr int PWM_FREQUENCY = 50;
constexpr int DUTY_CYCLE = 50;

constexpr int MOTOR_A1_PIN = 18;  // GPIO 18
constexpr int MOTOR_A2_PIN = 5;   // GPIO 5
constexpr int MOTOR_B1_PIN = 12;  // GPIO 12
constexpr int MOTOR_B2_PIN = 6;   // GPIO 6

constexpr int PWM_RANGE = 100;
constexpr int PWM_CLOCK = 384;
constexpr int PWM_STOP = 0;

#endif