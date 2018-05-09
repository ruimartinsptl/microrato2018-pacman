#include <Arduino.h>

#define Serial SerialUSB

#define PIN_WHITE_OUTPUT_LED 2
#define PIN_FLOOR_SENSOR A5

#define THRESHOULD_FLOOR_SENSOR 50

int floor_sensor_value;

void setup() {
    pinMode(PIN_WHITE_OUTPUT_LED, OUTPUT);
    pinMode(PIN_FLOOR_SENSOR, INPUT_PULLUP);

    Serial.begin(115200);
    while(!Serial); // Obrigatorio para o Arduino DUE

    // initialize digital pin LED_BUILTIN as an output.
    pinMode(LED_BUILTIN, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
    floor_sensor_value = analogRead(PIN_FLOOR_SENSOR);
    if (floor_sensor_value < THRESHOULD_FLOOR_SENSOR && floor_sensor_value < 500){
        digitalWrite(PIN_WHITE_OUTPUT_LED, HIGH);
        Serial.println(floor_sensor_value, DEC);
    }
    else {
        digitalWrite(PIN_WHITE_OUTPUT_LED, LOW);
        Serial.println(floor_sensor_value, DEC);
    }
    delay(10);
}