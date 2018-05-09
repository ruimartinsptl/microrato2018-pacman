//#define IR_PIN_INT
//#define IR_PIN_IRREMOTE
//#define IR_PIN_PWM

#include "pins.h"
#include <Arduino.h>
#include <Servo.h>

#ifdef IR_PIN_IRREMOTE
//#include <IRremote.h>
#endif
#ifdef IR_PIN_PWM
//#include "PWM.hpp"
#include "BeaconSensor.h"

#endif

//#if defined(ARDUINO_AVR_NANO) || defined(ARDUINO_AVR_ADK)
//#define IR_PIN 2
//#else
//#define Serial SerialUSB
//#define IR_PIN 22
//#endif

#define IR_PIN PIN_BEACON_FRONT

#ifdef IR_PIN_INT
void rising();
void falling();

boolean beacons_enabled = false;
volatile float pwm = 0;
volatile float period = 0;
volatile float p_period = 0;
volatile float freq = 0;
volatile uint32_t pwm_value_on = 0;
volatile uint32_t pwm_value_off = 0;
volatile uint32_t prev_time_on = 0;
volatile uint32_t prev_time_off = 0;

#endif

#ifdef IR_PIN_IRREMOTE
IRrecv irrecv(IR_PIN);
decode_results results;
#endif

Servo motor_servo_beacon;

#ifdef IR_PIN_PWM
//PWM pwm_front;
//PWM pwm_back;
//IR_PWM pwm_front;
//IR_PWM pwm_back;
#endif

// the setup function runs once when you press reset or power the board
void setup() {
    Serial.begin(115200);

//    while(!Serial);

#ifdef IR_PIN_IRREMOTE
    Serial.println("Enabling IRin");
    irrecv.enableIRIn(); // Start the receiver
    Serial.println("Enabled IRin");
#endif

    // initialize digital pin PIN_LED_RED as an output.
    pinMode(PIN_LED_RED, OUTPUT);
    pinMode(PIN_BEACONS_ENABLE, OUTPUT);
    pinMode(PIN_MOTOR_SERVO, OUTPUT);

#ifdef IR_PIN_INT
    pinMode(IR_PIN, INPUT);
    // when pin D22 goes high, call the rising function
    attachInterrupt(digitalPinToInterrupt(IR_PIN), rising, RISING);
//    attachInterrupt(0, falling, FALLING);
#endif

    // Servo
//    Serial.println("Servo...");
//    motor_servo_beacon.attach(PIN_MOTOR_SERVO);
//
//    Serial.println("0"); motor_servo_beacon.write(15);
//    delay(2000); Serial.println("0 OK");
//
//    Serial.println("180"); motor_servo_beacon.write(180);
//    delay(2000); Serial.println("180 OK");
//
//    Serial.println("90"); motor_servo_beacon.write(90);
//    delay(2000); Serial.println("90 OK");
//
//    Serial.println("Servo... OK\n");

//    int zero = 15;
//    for (int i = zero; i <= 180; i+=5) {
//        Serial.println(i); motor_servo_beacon.write(i);
//        delay(500); Serial.print(i); Serial.println(" OK");
//    }
//    Serial.println("90"); motor_servo_beacon.write(90);
//    delay(2000); Serial.println("90 OK");
//    motor_servo_beacon.detach();

#ifdef IR_PIN_PWM
//    pwm_front.begin(PIN_BEACON_FRONT, HIGH); // PWM on pin x reading PWM HIGH duration
//    pwm_back.begin(PIN_BEACON_FRONT, HIGH); // PWM on pin x reading PWM HIGH duration
#endif
    digitalWrite(PIN_BEACONS_ENABLE, HIGH);
}

String inString = "";

void beacon_calculate(uint32_t pwm_value_off, uint32_t pwm_value_on) {
    float pwm = 0;
    float period = 0;
    float freq = 0;
    period = pwm_value_off + pwm_value_on;
    freq = 1 / (period/1000000);
    pwm = (pwm_value_off / period);
    Serial.print("+ ");
    Serial.print(pwm_value_on);
    Serial.print("\t\t");
    Serial.print("- ");
    Serial.print(pwm_value_off);
    Serial.print("\t\t");
    Serial.print("P ");
    Serial.print((unsigned int) period);
    Serial.print("\t\t");
    Serial.print("F ");
    Serial.print(freq);
    Serial.print("\t\t");
    Serial.print("V ");
    Serial.print(pwm);
    Serial.println("");
}

// the loop function runs over and over again forever
void loop() {
    while (Serial.available() > 0) {
        int inChar = Serial.read();
        if (isDigit(inChar)) {
            // convert the incoming byte to a char and add it to the string:
            inString += (char)inChar;
        }
        // if you get a newline, print the string, then the string's value:
        if (inChar == '\n') {
            int value = inString.toInt();
            bool was_attached = motor_servo_beacon.attached();
            if (!was_attached) {
                motor_servo_beacon.attach(PIN_MOTOR_SERVO);
            }
            Serial.println(value); motor_servo_beacon.write(value);
            delay(2000); Serial.print(value); Serial.println(" OK");

            if (!was_attached) {
                motor_servo_beacon.detach();
            }
            // clear the string for new input:
            inString = "";
        }
    }

    digitalWrite(PIN_BEACONS_ENABLE, HIGH);
    delay(10);


#ifdef IR_PIN_INT
    pwm = 0;
    period = 0;
    p_period = 0;
    freq = 0;
    pwm_value_on = 0;
    pwm_value_off = 0;
    prev_time_on = 0;
    prev_time_off = 0;
    beacons_enabled = true;
#endif

#ifdef IR_PIN_IRREMOTE
    if (irrecv.decode(&results)) {
        Serial.println(results.value, HEX);
        irrecv.resume(); // Receive the next value
    }
    delay(100);
#endif

    Serial.print("*");
    digitalWrite(PIN_LED_RED, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(250);                       // wait for a second
    Serial.print(".");
    digitalWrite(PIN_LED_RED, LOW);    // turn the LED off by making the voltage LOW
    delay(250);                       // wait for a second
    Serial.println();

#ifdef IR_PIN_INT
//    period = pwm_value_off + pwm_value_on;
//    freq = 1 / (period/1000000);
//    pwm = (pwm_value_off / period);
//    if (p_period != period) {
//        Serial.print("+ ");
//        Serial.println(pwm_value_on);
//        Serial.print("- ");
//        Serial.println(pwm_value_off);
//        Serial.print("P ");
//        Serial.println(period);
//        Serial.print("F ");
//        Serial.println(freq);
//        Serial.print("V ");
//        Serial.println(pwm);
//        Serial.println("");
//        p_period = period;
//    }
    beacon_calculate(pwm_value_off, pwm_value_on);
    beacons_enabled = false;
#endif


#ifdef IR_PIN_PWM
    IR_PWM pwm_front;
    pwm_front.begin(PIN_BEACON_FRONT); // PWM on pin x reading PWM HIGH duration
    delay(1000);
    for (int i = 0; i < 4; i++) {
//        Serial.print("F "); Serial.println(pwm_front.getValue());
        Serial.print("F ");
        beacon_calculate(pwm_front.getValueLow(), pwm_front.getValueHigh());
        delay(100);
    }
    IR_PWM pwm_back;
    pwm_back.begin(PIN_BEACON_BACK); // PWM on pin x reading PWM HIGH duration
    delay(1000);
    for (int i = 0; i < 4; i++) {
//        Serial.print("B "); Serial.println(pwm_back.getValue());
        Serial.print("B ");
        beacon_calculate(pwm_back.getValueLow(), pwm_back.getValueHigh());
        delay(100);
    }

#endif
    delay(10);
    digitalWrite(PIN_BEACONS_ENABLE, LOW);
    delay(100);
}

#ifdef IR_PIN_INT

void rising() {
    attachInterrupt(digitalPinToInterrupt(IR_PIN), falling, FALLING);
    if (!beacons_enabled) return;
    prev_time_on = micros();

    pwm_value_off = micros() - prev_time_off;

//    Serial.print("-");
//    Serial.println(pwm_value_off);
}

void falling() {
    attachInterrupt(digitalPinToInterrupt(IR_PIN), rising, RISING);
    if (!beacons_enabled) return;
    prev_time_off = micros();

    pwm_value_on = micros() - prev_time_on;

//    Serial.print("+");
//    Serial.println(pwm_value_on);
}

#endif