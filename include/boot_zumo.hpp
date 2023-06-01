#ifndef BOOT_ZUMO
#define BOOT_ZUMO

#include <iostream>
#include <chrono>
#include <thread>
#include <wiringPi.h>

constexpr int PWM_FREQUENCY = 50;
constexpr int DUTY_CYCLE = 50;

constexpr int MOTOR_A1_PIN = 1;   // GPIO 18
constexpr int MOTOR_A2_PIN = 24;  // GPIO 5
constexpr int MOTOR_B1_PIN = 26;  // GPIO 12
constexpr int MOTOR_B2_PIN = 27;  // GPIO 6

#endif