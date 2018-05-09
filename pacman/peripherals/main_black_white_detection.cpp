#include <Arduino.h>

#define Serial SerialUSB

#define PIN_WHITE_OUTPUT_LED 2
#define PIN_FLOOR_SENSOR A5

#define THRESHOULD_FLOOR_SENSOR 50

int floor_sensor_value;

int get_floor_sensor_value(){
    return analogRead(PIN_FLOOR_SENSOR);
}

boolean is_black_on_floor(){
    return get_floor_sensor_value() < THRESHOULD_FLOOR_SENSOR;
}

void setup() {
    Serial.begin(115200);
    while(!Serial); // Obrigatorio para o Arduino DUE
}

void loop() {
    Serial.print(is_black_on_floor(), DEC); Serial.print(' '); Serial.println(get_floor_sensor_value(), DEC);
    delay(10);
}