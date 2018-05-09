
#include "FloorSensor.h"

FloorSensor::FloorSensor(uint8_t analog_pin, int threshold) {
    FloorSensor::analog_pin = analog_pin;
    FloorSensor::threshold = threshold;
    pinMode(analog_pin, INPUT);
}

int FloorSensor::get_value() {
    value = analogRead(analog_pin);
    return value;
}

bool FloorSensor::is_black_on_floor(){
    return get_value() > threshold;  // Branco = 0; Preto = 1024
}

int FloorSensor::get_threshold(){
    return threshold;
}
