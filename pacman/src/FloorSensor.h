
#ifndef PACMAN_FLOOR_SENSOR_H
#define PACMAN_FLOOR_SENSOR_H

#include <Arduino.h>

class FloorSensor {
private:
    uint8_t analog_pin;
    int value;
    int threshold;
public:
    FloorSensor(uint8_t analog_pin, int threshold);

    int get_value();
    bool is_black_on_floor();
    int get_threshold();
};

#endif //PACMAN_FLOOR_SENSOR_H
