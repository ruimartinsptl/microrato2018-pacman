//
// Created by Pedro Costa on 06/05/18.
//


#ifndef PACMAN_BeaconSensor_H
#define PACMAN_BeaconSensor_H

#include <Arduino.h>
#include <Servo.h>

/*This library uses interrupts to read PWM values. It is only intended to be used with Arduino Uno or Mega boards.
 * It is capable of reading high pulse (normal pwm) or low pulse (inverted pwm).
 *
 * Usage:
 * 1. Copy PWM.cpp and PWM.hpp in your project folder.
 * 1. Include PWM.hpp in your program. example #include "PWM.hpp"
 * 2. Create a PWM instance variable and call it what you want. example: PWM my_pwm;
 * 3. Initialise PWM instance you made and provide pin and trigger condition. example my_pwm.begin(2, 1);
 * 4. Call getValue to get PWM value. example *some_variable* = my_pwm.getValue();
 */

// IR_PWM class
class IR_PWM {
    byte pin;

    float pwm = 0;
    float period = 0;
    float freq = 0;
    unsigned int low = 0;
    unsigned int high = 0;

    float min_freq_threshold;
    float max_freq_threshold;
    float min_pwm_threshold;
    float max_pwm_threshold;

public:
    virtual ~IR_PWM();

    void begin(byte pin,
               float beacon_min_freq_threshold,
               float beacon_max_freq_threshold,
               float beacon_min_pwm_threshold,
               float beacon_max_pwm_threshold);
    void begin(byte pin);
    void begin();
    // pin = the digital pin to attach interrupt
    // On Arduino Uno, Nano, Mini, other 328-based, only pins 2 & 3 are usable for interrupts
    // On Arduino Mega, Mega2560, MegaADK, only pins 2, 3, 18, 19, 20 & 21 are usable for interrupts
    // State = 1 => the library will measure the duration of HIGH pulse (normal pwm)
    // State = 0 => the library will measure the duration of LOW pulse (inverted pwm)
    unsigned int getValueHigh();
    unsigned int getValueLow();
    // returns the pwm value
    void calculateValues();
    void pintValues();
    bool isValid();
};

class BeaconSensor {
    byte pin_beacon_front;
    byte pin_beacon_back;
    byte pin_beacons_enable;
    byte pin_motor_servo;

    float beacon_min_freq_threshold;
    float beacon_max_freq_threshold;
    float beacon_min_pwm_threshold;
    float beacon_max_pwm_threshold;

    int motor_servo_0;
    int motor_servo_180;

    IR_PWM pwm_front;
    IR_PWM pwm_back;

    Servo motor_servo;

    void _motor_servo_attach();
    int _motor_servo_write(int value);
    int _motor_servo_write_raw(int value);

public:
    BeaconSensor(byte pin_beacon_front,
                 byte pin_beacon_back,
                 byte pin_beacons_enable,
                 byte pin_motor_servo,
                 float beacon_min_freq_threshold,
                 float beacon_max_freq_threshold,
                 float beacon_min_pwm_threshold,
                 float beacon_max_pwm_threshold,
                 int motor_servo_0,
                 int motor_servo_180);

    void begin();

    int find(int start=0, int end=180, int step=5);


    int motor_servo_write(int value);
    int motor_servo_write_raw(int value);
};

#endif //PACMAN_BeaconSensor_H
