
#ifndef PACMAN_PINS_H
#define PACMAN_PINS_H

#ifndef A8
#define A8 8 // Isto é para enganar o IDE para não se queixar que não existe o PIN A11
#endif

// PINS
#define PIN_BEACONS_ENABLE 4  // Ligar os sensores da torre
#define PIN_BEACON_FRONT 2  // Sensor da torre frente
#define PIN_BEACON_BACK 3  // Sensor da torre chão
#define PIN_STOP 18 // INTERRUPTOR STOP [USA INTERRUPÇÕES]  // NOTA, NO DUE todos os pinos aceitam interrupções, mas no MEGA o pin 4 não aceita interrupções
#define PIN_START 5 // INTERRUPTOR START
#define PIN_LED_RED 6
#define PIN_MOTOR_SERVO 7  // Motor para o farol [USA PWM]
#define PIN_MOTORS_EN 8  // Activar os dois motores
#define PIN_MOTOR_LEFT_DIR 9  // Motor Esquerdo Direção
#define PIN_MOTOR_LEFT_STEP 10    // Motor Esquerdo passo
#define PIN_MOTOR_RIGHT_DIR 11  // Motor Direito Direção
#define PIN_MOTOR_RIGHT_STEP 12  // Motor Direito passo
//#define PIN_BEACON_EN1 14  //Sensor ON/Off
//#define PIN_BEACON_EN2 15  //Sensor ON/Off
//#define PIN_DEBUG 21  //PIN DE DEBUG
//
#define PIN_TRIG_LEFT   45  // Sensor de distancia LEFT
#define PIN_ECHO_LEFT   44  // Sensor de distancia LEFT
#define PIN_TRIG_FRONTL 47  // Sensor de distancia RIGHT
#define PIN_ECHO_FRONTL 46  // Sensor de distancia RIGHT
#define PIN_TRIG_FRONTR 49  // Sensor de distancia FRONT
#define PIN_ECHO_FRONTR 48  // Sensor de distancia FRONT
#define PIN_TRIG_RIGHT  51  // Sensor de distancia FRONT
#define PIN_ECHO_RIGHT  50  // Sensor de distancia FRONT
#define PIN_FLOOR_SENSOR  A8  // Sensor do chão

#endif //PACMAN_PINS_H
