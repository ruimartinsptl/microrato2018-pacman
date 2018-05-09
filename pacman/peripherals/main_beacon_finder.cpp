#include "pins.h"
#include "constants.h"
#include <Arduino.h>

#include "BeaconSensor.h"

BeaconSensor beacon(PIN_BEACON_FRONT, PIN_BEACON_BACK,
                    PIN_BEACONS_ENABLE, PIN_MOTOR_SERVO,
                    BEACON_MIN_FREQ_THRESHOLD,
                    BEACON_MAX_FREQ_THRESHOLD,
                    BEACON_MIN_PWM_THRESHOLD,
                    BEACON_MAX_PWM_THRESHOLD,
                    MOTOR_SERVO_0,
                    MOTOR_SERVO_180);
void setup() {
    Serial.begin(115200);

    while(!Serial);

    // initialize digital pin PIN_LED_RED as an output.
    pinMode(PIN_LED_RED, OUTPUT);

    beacon.begin();
}

String inString = "";
int c = 0;
int beacon_angle = 0;
#define NORMAL 0
#define CALIBRATION 1
int state = NORMAL;
// the loop function runs over and over again forever
void loop() {
    while (Serial.available() > 0) {
        int inChar = Serial.read();
        if (isDigit(inChar)) {
            // convert the incoming byte to a char and add it to the string:
            inString += (char)inChar;
        } else if (isAlpha(inChar)){
            if (inChar == 'c' || inChar == 'C') {
                state = CALIBRATION;
            } else if (inChar == 'r' || inChar == 'R') {
                state = NORMAL;
            }
        }
        // if you get a newline, print the string, then the string's value:
        if (inChar == '\n') {
            int value = (int) inString.toInt();
            if (c == 0) {
                Serial.print("Cal ");
                beacon.motor_servo_write(value);
                c = 1;
            } else if (c == 1) {
                Serial.print("Raw ");
                beacon.motor_servo_write_raw(value);
                c = 0;
            }
            // clear the string for new input:
            inString = "";
        }
    }

    if (state == NORMAL) {
        Serial.print("*");
        digitalWrite(PIN_LED_RED, HIGH);   // turn the LED on (HIGH is the voltage level)
        delay(250);                       // wait for a second
        Serial.print(".");
        digitalWrite(PIN_LED_RED, LOW);    // turn the LED off by making the voltage LOW
        delay(250);                       // wait for a second
        Serial.println();

        beacon_angle = beacon.find();
        if (beacon_angle != -1) {
            Serial.print("Found at ");
            Serial.println(beacon_angle);
            delay(1000);
        }
    }
}

