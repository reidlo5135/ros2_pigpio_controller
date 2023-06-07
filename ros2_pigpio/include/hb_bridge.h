#ifndef HB_BRIDGE_H
#define HB_BRIDGE_H

#include <pigpiod_if2.h>

int pi_;
int pin_in1_;
int pin_in2_;
int pin_pwm_;
int type_;

void hb_pwm_off(int pi, int pin_in1, int pin_in2, int pin_pwm) {
    set_mode(pi, pin_in1, PI_INPUT);
    set_mode(pi, pin_in2, PI_INPUT);
    if (pin_pwm_ >= 0) {
        set_mode(pi, pin_pwm, PI_INPUT);
    }
}

void hb_set_pin(int pi, int pin_in1, int pin_in2) {
    pi_ = pi;
    pin_in1_ = pin_in1;
    pin_in2_ = pin_in2;

    type_ = 1;

    set_mode(pi_, pin_in1_, PI_OUTPUT);
    set_mode(pi_, pin_in2_, PI_OUTPUT);
}

void hb_set_pin_pwm(int pi, int pin_in1, int pin_in2, int pin_pwm) {
    pi_ = pi;
    pin_in1_ = pin_in1;
    pin_in2_ = pin_in2;
    pin_pwm_ = pin_pwm;

    type_ = 0;

    set_mode(pi_, pin_in1_, PI_OUTPUT);
    set_mode(pi_, pin_in2_, PI_OUTPUT);
    set_mode(pi_, pin_pwm_, PI_OUTPUT);
}

void hb_drive(int power) {
    switch (type_) {
        case 0:
            if (power > 0) {
                gpio_write(pi_, pin_in1_, 1);
                gpio_write(pi_, pin_in2_, 0);
            } else {
                gpio_write(pi_, pin_in1_, 0);
                gpio_write(pi_, pin_in2_, 1);
                power = -power;
            }
            set_PWM_dutycycle(pi_, pin_pwm_, power);
            break;

        case 1:
            if (power > 0) {
                set_PWM_dutycycle(pi_, pin_in1_, power);
                gpio_write(pi_, pin_in2_, 0);
            } else {
                power = -power;
                gpio_write(pi_, pin_in1_, 0);
                set_PWM_dutycycle(pi_, pin_in2_, power);
            }
            break;
    }
}

#endif