//
// Created by Pedro Costa on 06/05/18.
//

#include "BeaconSensor.h"

// Global variables for ISR routines
unsigned int value_HIGH[6];
unsigned int value_LOW[6];
unsigned long timers_HIGH[6];
unsigned long timers_LOW[6];
bool ISR_enabled = false;

void ISR_2_rising();
void ISR_2_falling();
void ISR_3_rising();
void ISR_3_falling();
void ISR_18_rising();
void ISR_18_falling();
void ISR_19_rising();
void ISR_19_falling();
void ISR_20_rising();
void ISR_20_falling();
void ISR_21_rising();
void ISR_21_falling();

// ISR for pin 2
void ISR_2_rising() {
    attachInterrupt(digitalPinToInterrupt(2), ISR_2_falling, FALLING);
    if (!ISR_enabled) return;
    unsigned long now = micros();
    timers_HIGH[0] = now;
    value_LOW[0] = (unsigned int) (now - timers_LOW[0]);
}
void ISR_2_falling() {
    attachInterrupt(digitalPinToInterrupt(2), ISR_2_rising, RISING);
    if (!ISR_enabled) return;
    unsigned long now = micros();
    timers_LOW[0] = now;
    value_HIGH[0] = (unsigned int) (now - timers_HIGH[0]);
}

// ISR for pin 3
void ISR_3_rising() {
    attachInterrupt(digitalPinToInterrupt(3), ISR_3_falling, FALLING);
    if (!ISR_enabled) return;
    unsigned long now = micros();
    timers_HIGH[1] = now;
    value_LOW[1] = (unsigned int) (now - timers_LOW[1]);
}
void ISR_3_falling() {
    attachInterrupt(digitalPinToInterrupt(3), ISR_3_rising, RISING);
    if (!ISR_enabled) return;
    unsigned long now = micros();
    timers_LOW[1] = now;
    value_HIGH[1] = (unsigned int) (now - timers_HIGH[1]);
}

// ISR for pin 18
void ISR_18_rising() {
    attachInterrupt(digitalPinToInterrupt(18), ISR_18_falling, FALLING);
    if (!ISR_enabled) return;
    unsigned long now = micros();
    timers_HIGH[2] = now;
    value_LOW[2] = (unsigned int) (now - timers_LOW[2]);
}
void ISR_18_falling() {
    attachInterrupt(digitalPinToInterrupt(18), ISR_18_rising, RISING);
    if (!ISR_enabled) return;
    unsigned long now = micros();
    timers_LOW[2] = now;
    value_HIGH[2] = (unsigned int) (now - timers_HIGH[2]);
}

// ISR for pin 19
void ISR_19_rising() {
    attachInterrupt(digitalPinToInterrupt(19), ISR_19_falling, FALLING);
    if (!ISR_enabled) return;
    unsigned long now = micros();
    timers_HIGH[3] = now;
    value_LOW[3] = (unsigned int) (now - timers_LOW[3]);
}
void ISR_19_falling() {
    attachInterrupt(digitalPinToInterrupt(19), ISR_19_rising, RISING);
    if (!ISR_enabled) return;
    unsigned long now = micros();
    timers_LOW[3] = now;
    value_HIGH[3] = (unsigned int) (now - timers_HIGH[3]);
}

// ISR for pin 20
void ISR_20_rising() {
    attachInterrupt(digitalPinToInterrupt(20), ISR_20_falling, FALLING);
    if (!ISR_enabled) return;
    unsigned long now = micros();
    timers_HIGH[4] = now;
    value_LOW[4] = (unsigned int) (now - timers_LOW[4]);
}
void ISR_20_falling() {
    attachInterrupt(digitalPinToInterrupt(20), ISR_20_rising, RISING);
    if (!ISR_enabled) return;
    unsigned long now = micros();
    timers_LOW[4] = now;
    value_HIGH[4] = (unsigned int) (now - timers_HIGH[4]);
}
// ISR for pin 21
void ISR_21_rising() {
    attachInterrupt(digitalPinToInterrupt(21), ISR_21_falling, FALLING);
    if (!ISR_enabled) return;
    unsigned long now = micros();
    timers_HIGH[5] = now;
    value_LOW[5] = (unsigned int) (now - timers_LOW[5]);
}
void ISR_21_falling() {
    attachInterrupt(digitalPinToInterrupt(21), ISR_21_rising, RISING);
    if (!ISR_enabled) return;
    unsigned long now = micros();
    timers_LOW[5] = now;
    value_HIGH[5] = (unsigned int) (now - timers_HIGH[5]);
}

// Begin function to setup interrupt and attach ISR
void IR_PWM::begin(byte pin,
                   float min_freq_threshold,
                   float max_freq_threshold,
                   float min_pwm_threshold,
                   float max_pwm_threshold) {
    IR_PWM::min_freq_threshold = min_freq_threshold;
    IR_PWM::max_freq_threshold = max_freq_threshold;
    IR_PWM::min_pwm_threshold = min_pwm_threshold;
    IR_PWM::max_pwm_threshold = max_pwm_threshold;
    begin(pin);
}

void IR_PWM::begin() {
    begin(pin);
}

void IR_PWM::begin(byte pin) {
    IR_PWM::pin = pin;
    switch (pin) {
        case 2:
            value_HIGH[0] = 0;
            value_LOW[0] = 0;
            timers_HIGH[0] = 0;
            timers_LOW[0] = 0;
            attachInterrupt(digitalPinToInterrupt(pin), ISR_2_rising, RISING);
            break;
        case 3:
            value_HIGH[1] = 0;
            value_LOW[1] = 0;
            timers_HIGH[1] = 0;
            timers_LOW[1] = 0;
            attachInterrupt(digitalPinToInterrupt(pin), ISR_3_rising, RISING);
            break;
        case 18:
            value_HIGH[2] = 0;
            value_LOW[2] = 0;
            timers_HIGH[2] = 0;
            timers_LOW[2] = 0;
            attachInterrupt(digitalPinToInterrupt(pin), ISR_18_rising, RISING);
            break;
        case 19:
            value_HIGH[3] = 0;
            value_LOW[3] = 0;
            timers_HIGH[3] = 0;
            timers_LOW[3] = 0;
            attachInterrupt(digitalPinToInterrupt(pin), ISR_19_rising, RISING);
            break;
        case 20:
            value_HIGH[4] = 0;
            value_LOW[4] = 0;
            timers_HIGH[4] = 0;
            timers_LOW[4] = 0;
            attachInterrupt(digitalPinToInterrupt(pin), ISR_20_rising, RISING);
            break;
        case 21:
            value_HIGH[5] = 0;
            value_LOW[5] = 0;
            timers_HIGH[5] = 0;
            timers_LOW[5] = 0;
            attachInterrupt(digitalPinToInterrupt(pin), ISR_21_rising, RISING);
            break;
        default:
            break;
    }
}


// Function to return channel value
unsigned int IR_PWM::getValueHigh() {
    switch (pin) {
        case 2:
            return value_HIGH[0];
        case 3:
            return value_HIGH[1];
        case 18:
            return value_HIGH[2];
        case 19:
            return value_HIGH[3];
        case 20:
            return value_HIGH[4];
        case 21:
            return value_HIGH[5];
        default:
            break;
    }
}

unsigned int IR_PWM::getValueLow() {
    switch (pin) {
        case 2:
            return value_LOW[0];
        case 3:
            return value_LOW[1];
        case 18:
            return value_LOW[2];
        case 19:
            return value_LOW[3];
        case 20:
            return value_LOW[4];
        case 21:
            return value_LOW[5];
        default:
            break;
    }
}

void IR_PWM::calculateValues() {
    low = getValueLow();
    high = getValueHigh();
    period = low + getValueHigh();
    freq = 1 / (period/1000000);
    pwm = (low / period);
}

void IR_PWM::pintValues() {
    Serial.print("+ ");
    Serial.print(high);
    Serial.print("\t\t");
    Serial.print("- ");
    Serial.print(low);
    Serial.print("\t\t");
    Serial.print("P ");
    Serial.print((unsigned int) period);
    Serial.print("\t\t");
    Serial.print("F ");
    Serial.print(freq);
    Serial.print("\t\t");
    Serial.print("V ");
    Serial.print(pwm);

//    Serial.print(" (");
//    Serial.print(freq);
//    Serial.print(" ");
//    Serial.print(min_freq_threshold);
//    Serial.print(" ");
//    Serial.print(max_freq_threshold);
//    Serial.print(" ");
//    Serial.print(pwm);
//    Serial.print(" ");
//    Serial.print(min_pwm_threshold);
//    Serial.print(" ");
//    Serial.print(max_pwm_threshold);
//    Serial.print(") ");

    Serial.print("\t\t");
    Serial.print(" ");
    Serial.print(isValid());
    Serial.println("");
}

IR_PWM::~IR_PWM() {
    detachInterrupt(digitalPinToInterrupt(pin));
}

bool IR_PWM::isValid() {
    bool ret = false;

    if (freq > min_freq_threshold && freq < max_freq_threshold) {
        if (pwm > min_pwm_threshold && pwm < max_pwm_threshold) {
            ret = true;
        }
    }
    return ret;
}

BeaconSensor::BeaconSensor(
        byte pin_beacon_front,
        byte pin_beacon_back,
        byte pin_beacons_enable,
        byte pin_motor_servo,
        float beacon_min_freq_threshold,
        float beacon_max_freq_threshold,
        float beacon_min_pwm_threshold,
        float beacon_max_pwm_threshold,
        int motor_servo_0,
        int motor_servo_180) :
            pin_beacon_front(pin_beacon_front),
            pin_beacon_back(pin_beacon_back),
            pin_beacons_enable(pin_beacons_enable),
            pin_motor_servo(pin_motor_servo),
            beacon_min_freq_threshold(beacon_min_freq_threshold),
            beacon_max_freq_threshold(beacon_max_freq_threshold),
            beacon_min_pwm_threshold(beacon_min_pwm_threshold),
            beacon_max_pwm_threshold(beacon_max_pwm_threshold),
            motor_servo_0(motor_servo_0),
            motor_servo_180(motor_servo_180){

}

void BeaconSensor::begin() {
    pinMode(pin_beacon_front, INPUT);
    pinMode(pin_beacon_back, INPUT);
    pinMode(pin_beacons_enable, OUTPUT);
    pinMode(pin_motor_servo, OUTPUT);

    pwm_front.begin(pin_beacon_front,
                    beacon_min_freq_threshold, beacon_max_freq_threshold,
                    beacon_min_pwm_threshold, beacon_max_pwm_threshold);
    pwm_back.begin(pin_beacon_back,
                   beacon_min_freq_threshold, beacon_max_freq_threshold,
                   beacon_min_pwm_threshold, beacon_max_pwm_threshold);


    _motor_servo_attach();
    _motor_servo_write(90);
    delay(500);
    motor_servo.detach();
}


void BeaconSensor::_motor_servo_attach() {
    if (motor_servo_0 <= 180) {
        motor_servo.attach(pin_motor_servo);
    } else {
        motor_servo.attach(pin_motor_servo, motor_servo_0, motor_servo_180);
    }
}

int BeaconSensor::motor_servo_write(int value) {
    int _value;
    bool was_attached = motor_servo.attached();
    if (!was_attached) {
        _motor_servo_attach();
    }
//    Serial.print(value);
//    Serial.print(" ");
    _value = _motor_servo_write(value);
//    Serial.print(_value);
//    Serial.print(" ");
//    Serial.print(motor_servo.m_min);
//    Serial.print(" ");
//    Serial.print(motor_servo.m_max);
//    Serial.print(" ");
//    Serial.print(motor_servo.m_value);
    delay(1000);
//    Serial.println(" OK");
    if (!was_attached) {
        motor_servo.detach();
    }
    return _value;
}
int BeaconSensor::motor_servo_write_raw(int value) {
    int _value;
    bool was_attached = motor_servo.attached();
    if (!was_attached) {
        _motor_servo_attach();
    }
//    Serial.print(value);
//    Serial.print(" ");
    _value = _motor_servo_write_raw(value);
//    Serial.print(_value);
//    Serial.print(" ");
//    Serial.print(motor_servo.m_min);
//    Serial.print(" ");
//    Serial.print(motor_servo.m_max);
//    Serial.print(" ");
//    Serial.print(motor_servo.m_value);
    delay(1000);
//    Serial.println(" OK");
    if (!was_attached) {
        motor_servo.detach();
    }
    return _value;
}

int BeaconSensor::_motor_servo_write(int value) {
    int _value;
    if (motor_servo_0 <= 180) {
        _value = map(value, 0, 180, motor_servo_0, motor_servo_180);
    } else {
        _value = value;
    }
    motor_servo.write(_value);
    return _value;
}

int BeaconSensor::_motor_servo_write_raw(int value) {
    motor_servo.write(value);
    return value;
}

int BeaconSensor::find(int start, int end, int step) {
    int temp, i;
    if (start > end) {
        temp = start;
        start = end;
        end = temp;
    }
    _motor_servo_attach();
    Serial.println("Beacon find...");
    for (i = start; i <= end; i+=step) {
        pwm_front.begin();
        pwm_back.begin();
        digitalWrite(pin_beacons_enable, HIGH);
//        Serial.print(i);
//        Serial.print(" ");
//        Serial.print(_motor_servo_write(i));
        _motor_servo_write(i);
//        _motor_servo_write(i);
        delay(200);
        ISR_enabled = true;
//        Serial.println(" OK");
        delay(50);
        pwm_front.calculateValues();
        if (pwm_front.isValid()) {
            Serial.print("F ");
            pwm_front.pintValues();
            Serial.println("Found beacon front");
            ISR_enabled = false;
            motor_servo.detach();

            if (step > 2) {
                temp = find(i + 2, i + 2, 2);
                if (temp != -1) {
                    return temp;
                } else {
                    _motor_servo_attach();
                }
            } else {
                return i;
            }
        }

        pwm_back.calculateValues();
        if (pwm_back.isValid()) {
            Serial.print("B ");
            pwm_back.pintValues();
            Serial.println("Found beacon back");
            ISR_enabled = false;
            motor_servo.detach();

            if (step > 2) {
                temp = find(i + 2, i + 2, 2);
                if (temp != -1) {
                    return temp;
                } else {
                    _motor_servo_attach();
                }
            } else {
                return i + 180;
            }
        }

        ISR_enabled = false;
        digitalWrite(pin_beacons_enable, LOW);
        delay(10);
    }

    if (step > 2) {
        _motor_servo_write(90);
        delay(500);
    }
    motor_servo.detach();
    return -1;
}


