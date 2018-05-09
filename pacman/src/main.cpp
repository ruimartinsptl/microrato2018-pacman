
// Este documento está pré-preenchido mas não contem o algoritmo usado no concurso.
// Serve de exemplo.

#include <Arduino.h>
#include <Servo.h>
#include <AccelStepper.h>
#include <NewPing.h>
#include "pins.h"
#include "constants.h"
#include "state_machine.h"
#include "motors_steper.h"
#include "led_red.h"
#include "FloorSensor.h"
#include "BeaconSensor.h"

#if defined(ARDUINO_AVR_NANO) || defined(ARDUINO_AVR_ADK) || defined(ARDUINO_AVR_MEGA2560)

#else
// XPTO
// DUE
// Comentar o seguinte define quando usar a porta de programming, descomentar quando se usar a porta USB nativa
#define Serial SerialUSB
// #define Serial Serial1
#endif
#define ENABLE_DEBUG


// (...YOUR CODE HERE...)

AccelStepper stepper_left(AccelStepper::DRIVER, PIN_MOTOR_LEFT_STEP, PIN_MOTOR_LEFT_DIR);
AccelStepper stepper_right(AccelStepper::DRIVER, PIN_MOTOR_RIGHT_STEP, PIN_MOTOR_RIGHT_DIR);

// NewPing setup of pins and maximum distance.
NewPing sonar_left(PIN_TRIG_LEFT, PIN_ECHO_LEFT, PING_MAX_DISTANCE / 10);
NewPing sonar_front_left(PIN_TRIG_FRONTL, PIN_ECHO_FRONTL, PING_MAX_DISTANCE / 10);
NewPing sonar_front_right(PIN_TRIG_FRONTR, PIN_ECHO_FRONTR, PING_MAX_DISTANCE / 10);
NewPing sonar_right(PIN_TRIG_RIGHT, PIN_ECHO_RIGHT, PING_MAX_DISTANCE / 10);

FloorSensor floor_sensor(PIN_FLOOR_SENSOR, THRESHOLD_FLOOR_SENSOR);

BeaconSensor beacon(PIN_BEACON_FRONT, PIN_BEACON_BACK,
                    PIN_BEACONS_ENABLE, PIN_MOTOR_SERVO,
                    BEACON_MIN_FREQ_THRESHOLD,
                    BEACON_MAX_FREQ_THRESHOLD,
                    BEACON_MIN_PWM_THRESHOLD,
                    BEACON_MAX_PWM_THRESHOLD,
                    MOTOR_SERVO_0,
                    MOTOR_SERVO_180);

// (...YOUR CODE HERE...)

uint32_t distance_left = 0, distance_right = 0, distance_frontL = 0, distance_frontR = 0;
int beacon_angle = 0;
bool led_is_on = false;
bool beacon_front_is_on = false;
bool beacon_back_is_on = false;
uint8_t id_sensor_to_read = 0;

// (...YOUR CODE HERE...)

void kill_motors() {
    stepper_left.stop();
    stepper_right.stop();
}

void refresh_all_distance_sensors() {
    distance_left = (sonar_left.ping_cm() * 10);
    distance_right = (sonar_right.ping_cm() * 10);
    distance_frontR = (sonar_front_right.ping_cm() * 10);
    distance_frontL = (sonar_front_left.ping_cm() * 10);
}


void refresh_one_distance_sensors(uint8_t id) {
//    Serial.print("SENSOR: "); Serial.println(id);
    switch (id) {
        case 1:
            distance_left = sonar_left.ping_cm() * 10;
            break;
        case 2:
            distance_frontL = sonar_front_left.ping_cm() * 10;
            break;
        case 3:
            distance_frontR = sonar_front_right.ping_cm() * 10;
            break;
        case 4:
            distance_right = sonar_right.ping_cm() * 10;
            break;
    }
}


void do_things_on_cheese() {
    // (...YOUR CODE HERE...)
}

void sinalize_end_with_led() {
    // (...YOUR CODE HERE...)
}

// TEST ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void test_all() {
    Serial.println("Running test_all()...");

    // LED
    Serial.println("LED...");
    for (int i = 0; i < 10; i++) {
        if (i % 2 == 0)
            turn_on_led_red();
        else
            turn_off_led_red();
        delay(150);
    }
    Serial.println("LED... OK\n");

    // Main motors
    Serial.println("Motors...");
    turn_on_main_motors();
    stepper_left.setCurrentPosition(0);
    stepper_right.setCurrentPosition(0);
    aux1 = stepper_left.currentPosition() + PERIMETER_ROBOT_PERIMETER / 2 * STEPS_MM;
    aux2 = stepper_right.currentPosition() - PERIMETER_ROBOT_PERIMETER / 2 * STEPS_MM;
    stepper_left.moveTo(aux1);
    stepper_right.moveTo(aux2);
    while (mouse_state != STATE_MOUSE_ABORTED and
           (stepper_left.currentPosition() < (aux1) or stepper_right.currentPosition() > (aux2))) {
        stepper_left.run();
        stepper_right.run();
    }

    // Main motors
    for (uint_fast32_t i = 100000; i > 0; i--) {
        stepper_left.run();
        stepper_right.run();
    }
    turn_off_main_motors();
    Serial.println("Motors... OK\n");

    // Servo
    Serial.println("Servo...");
    beacon.motor_servo_write(0);
    beacon.motor_servo_write(180);
    beacon.motor_servo_write(90);
    Serial.println("Servo... OK\n");

    // Sensores da frente
    Serial.println("Sensores distancia...");
    refresh_all_distance_sensors();
    Serial.print(" L: ");
    Serial.print(distance_left);
    Serial.println("mm");
    Serial.print("FL: ");
    Serial.print(distance_frontL);
    Serial.println("mm");
    Serial.print("FR: ");
    Serial.print(distance_frontR);
    Serial.println("mm");
    Serial.print(" R: ");
    Serial.print(distance_right);
    Serial.println("mm");
    Serial.println("Sensores distancia... OK\n");

    // Sensor do chão
    Serial.println("Sensor chão...");
    Serial.print("Chão: ");
    Serial.print(floor_sensor.get_value());
    Serial.print("\t is black:");
    Serial.println(floor_sensor.is_black_on_floor());
    Serial.println("Sensor chão... OK\n");

    Serial.println("Running test_all()... OK");
}

void test_inputs_all() {
    // Sensor do chão
    if (floor_sensor.is_black_on_floor()) {
        turn_on_led_red();
    } else {
        turn_off_led_red();
    }

    Serial.println("Sensor chão...");
    Serial.print("Chão: ");
    Serial.print(floor_sensor.get_value());
    Serial.print("\t is black:");
    Serial.println(floor_sensor.is_black_on_floor());
    Serial.println("Sensor chão... OK\n");

    // Sensores da frente
    Serial.println("Sensores distancia...");
    refresh_all_distance_sensors();
    Serial.print(" L: ");
    Serial.print(distance_left);
    Serial.println("mm");
    Serial.print("FL: ");
    Serial.print(distance_frontL);
    Serial.println("mm");
    Serial.print("FR: ");
    Serial.print(distance_frontR);
    Serial.println("mm");
    Serial.print(" R: ");
    Serial.print(distance_right);
    Serial.println("mm");
    Serial.println("Sensores distancia... OK\n");

    // Beacons
    Serial.println("Beacons...");
//    Serial.print("BEACON FRONT: "); Serial.println(analogRead(beacon_front));
//    Serial.print("BEACON BACK: "); Serial.println(analogRead(beacon_front));
    Serial.println("BEACON FRONT:");

    Serial.println("Running test_all()... OK");
}

// TEST ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// INTERRUPTIONS ///////////////////////////////////////////////////////////////////////////////////////////////////////
void int_stop_pressed() {  // ISR stop button
    // (...YOUR CODE HERE...)
}

// INTERRUPTIONS ///////////////////////////////////////////////////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void set_pins_mode() {
    // PINS
    pinMode(PIN_STOP, INPUT_PULLUP);
    pinMode(PIN_START, INPUT_PULLUP);
    pinMode(PIN_LED_RED, OUTPUT);

    pinMode(PIN_MOTORS_EN, OUTPUT);
    pinMode(PIN_MOTOR_LEFT_DIR, OUTPUT);
    pinMode(PIN_MOTOR_LEFT_STEP, OUTPUT);
    pinMode(PIN_MOTOR_RIGHT_DIR, OUTPUT);
    pinMode(PIN_MOTOR_RIGHT_STEP, OUTPUT);

    pinMode(PIN_TRIG_LEFT, OUTPUT);
    pinMode(PIN_ECHO_LEFT, INPUT);  //US SENSORS HC-SR04
    pinMode(PIN_TRIG_RIGHT, OUTPUT);
    pinMode(PIN_ECHO_RIGHT, INPUT);  //US SENSORS HC-SR04
    pinMode(PIN_TRIG_FRONTL, OUTPUT);
    pinMode(PIN_ECHO_FRONTL, INPUT);  //US SENSORS HC-SR04
    pinMode(PIN_TRIG_FRONTR, OUTPUT);
    pinMode(PIN_ECHO_FRONTR, INPUT);  //US SENSORS HC-SR04
////    pinMode(PIN_COLOR_FLOOR, INPUT);  // IR floor
}

void print_info() {
    Serial.print("BAUDRATE:\t");
    Serial.println(BAUDRATE);
    Serial.print("TICK_MAIN_PROCESS:\t\t");
    Serial.println(TICK_MAIN_PROCESS);
    Serial.print("TIMEOUT:\t");
    Serial.print(TIMEOUT / 1000);
    Serial.println(" s");
    Serial.print("STEPS_MM:\t");
    Serial.println(STEPS_MM);
    Serial.print("WHEEL_PERIMETER:\t");
    Serial.print(WHEEL_PERIMETER);
    Serial.println(" mm");
    Serial.print("WHEEL_RADIUS:\t");
    Serial.print(WHEEL_RADIUS);
    Serial.println(" mm");
}

void send_variables_to_serial(bool force = false) {
    currentMillis = millis();
    if (force || (currentMillis - previousMillis_t_print_to_serial) >= TICK_PRINT_TO_SERIAL) {
        previousMillis_t_print_to_serial = currentMillis;
        Serial.print("--> "); // START
        Serial.print(floor_sensor.get_value());
        Serial.print(" "); // Valor do sensor do chão
        Serial.print(floor_sensor.is_black_on_floor());
        Serial.print(" "); // É preto ou não
        Serial.print(distance_left);
        Serial.print(" "); // Valor sensor esquerdo
        Serial.print(distance_frontL);
        Serial.print(" "); // Valor sensor esquerdo frente
        Serial.print(distance_frontR);
        Serial.print(" "); // Valor sensor direito frente
        Serial.print(distance_right);
        Serial.print(" "); // Valor sensor direito
        Serial.print(beacon_angle);
        Serial.print(" "); // Angulo do farol
        Serial.print(current_position.x);
        Serial.print(";");
        Serial.print(current_position.y);
        Serial.print(";");
        Serial.print(current_position.theta); // em radianos
        Serial.print(" "); // Position
        Serial.print(led_is_on);
        Serial.print(" "); // Led está ligado?
        Serial.print(mouse_state);
        Serial.print(" "); // Estado do robot
        Serial.print(stepper_left.currentPosition());
        Serial.print(";");
        Serial.print(stepper_right.currentPosition());
        Serial.print(" ");
        Serial.print('\n'); // END
    }
}

void setup() {
#ifdef ENABLE_DEBUG
    Serial.println("Setup...");
#endif
    Serial.begin(BAUDRATE);
//    while(!Serial) {yield();} // Pode bloquear // XPTO // Descomentar quando tiver ligado por cabo
    set_pins_mode();

//    turn_off_beacon();
    turn_off_main_motors();
    turn_off_led_red();
//    turn_on_servo();
//    set_servo_degree(90);
//    turn_off_servo();

    stepper_left.setMaxSpeed(MOTORS_MAX_SPEED);
    stepper_left.setAcceleration(MOTORS_ACELERATION);
    stepper_right.setMaxSpeed(MOTORS_MAX_SPEED);
    stepper_right.setAcceleration(MOTORS_ACELERATION);
    stepper_right.setPinsInverted(true, false, false);

    beacon.begin();

    beacon.motor_servo_write(0);
    beacon.motor_servo_write(180);
    beacon.motor_servo_write(90);

    print_info();

//    test_all();

    attachInterrupt(digitalPinToInterrupt(PIN_STOP), int_stop_pressed,
                    FALLING); // RISING - to trigger when the pin goes from low to high; FALLING - for when the pin goes from high to low;
#ifdef ENABLE_DEBUG
    Serial.println("Setup... OK");
#endif
}

void loop() {
//    test_inputs_all();
//    test_all();
//    turn_on_led_red();
    mouse_state = STATE_MOUSE_WAITTING_TO_START;
    turn_off_led_red();
    turn_off_main_motors();
#ifdef ENABLE_DEBUG
    Serial.print("Waiting for START...");
#endif
    // WAITTING START BUTTON
    while (digitalRead(PIN_START) == HIGH) {
        currentMillis = millis();
        if ((currentMillis - previousMillis_t_read_sensors) >= TICK_READ_SENSORS) {
            previousMillis_t_read_sensors = currentMillis;
            refresh_all_distance_sensors();
        }

        send_variables_to_serial();
        yield();
    }

    // PIN_START pressed
    time_start = millis();
#ifdef ENABLE_DEBUG
    Serial.print("PIN_START pressed\n");
#endif
    turn_on_main_motors();
    stepper_left.stop();
    stepper_right.stop();
    stepper_left.setCurrentPosition(0);
    stepper_right.setCurrentPosition(0);

    beacon.motor_servo_write(90);
    mouse_state = STATE_MOUSE_SEARCH_BEACON;
    journey_state = STATE_JOURNEY_GOING_TO_CHEESE;

    // Main while inside loop()
    while (mouse_state != STATE_MOUSE_ALL_DONE && mouse_state != STATE_MOUSE_ABORTED) {

        // (...YOUR CODE HERE...)

        send_variables_to_serial();
        stepper_left.run();
        stepper_right.run();
    }

#ifdef ENABLE_DEBUG
    if (mouse_state == STATE_MOUSE_ALL_DONE) {
        Serial.print(
                "MAIN termina com sucesso, o rato fez tudo o que tinha a fazer. Rato vai ficar à espera de novo START");
    }

    if (mouse_state == STATE_MOUSE_ABORTED) {
        Serial.print("MAIN termina porque foi premido ABORT. Rato vai ficar de novo no estado à espera de START");
    }
#endif
}
