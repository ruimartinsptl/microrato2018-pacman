
#include "motors_steper.h"

void turn_off_main_motors(){
    digitalWrite(PIN_MOTORS_EN, HIGH);
}
void turn_on_main_motors(){
    digitalWrite(PIN_MOTORS_EN, LOW);
}