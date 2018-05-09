//
// Created by Rui Filipe de Sousa Martins on 01/05/2018.
//

#include "led_red.h"

void turn_on_led_red(){
    digitalWrite(PIN_LED_RED, HIGH);
}

void turn_off_led_red(){
    digitalWrite(PIN_LED_RED, LOW);
}

