
#include <Arduino.h>
#include <AccelStepper.h>

#define PIN_MOTOR_DIR 8  // Motor Direção
#define PIN_MOTOR_STEP 9    // Motor passo

#define MIN_DELAY_MOTORS 17000 // Microseconds
#define MOTOR_TOTAL_STEPS 200 // Microseconds
#define MOTOR_MICROSTEPS 8 // Microseconds

AccelStepper stepper(AccelStepper::DRIVER, PIN_MOTOR_STEP, PIN_MOTOR_DIR);
//AccelStepper(PIN_MOTOR_STEP, uint8_t pin1 = 2, uint8_t pin2 = 3, uint8_t pin3 = 4, uint8_t pin4 = 5);
//AccelStepper stepper(2, 10, 11);

int pos = 3600;

void setup() {
    stepper.setMaxSpeed(3000.0 * MOTOR_MICROSTEPS);
    stepper.setAcceleration(150.0 * MOTOR_MICROSTEPS);

//    stepper.setMaxSpeed(300);
//    stepper.setAcceleration(100);

//    stepper.setSpeed(50);
}

void loop() {
//    stepper.runSpeed();
//    stepper.run();

    if (stepper.distanceToGo() == 0) {
        delay(50);
        pos = -pos;
        stepper.moveTo(pos);
    }
    stepper.run();
}


