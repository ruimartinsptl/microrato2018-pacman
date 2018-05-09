
#ifndef PACMAN_CONSTANTS_H
#define PACMAN_CONSTANTS_H



#define TIMEOUT ((3*60*1000) - 4000) // 3 Minutos até timeout - 4 segundos



#define BAUDRATE 115200 // 115200 ; 2000000; 230400 ; 500000
#define TICK_MAIN_PROCESS 50 //t(ms)
#define TICK_READ_SENSORS 50 // t(ms) Na documentação diz para não fazer mais de 20 pings por segundo
#define TICK_PRINT_TO_SERIAL 500 // t(ms) Na documentação diz para não fazer mais de 20 pings por segundo
#define TICK_FIND_SENSOR 20 * 1000 // t(ms) Na documentação diz para não fazer mais de 20 pings por segundo

#define DISTANCE_BETWEEN_WHEELS 150 // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#define PERIMETER_ROBOT_PERIMETER (2*PI*DISTANCE_BETWEEN_WHEELS/2) // Perimeter between wheels
#define WHEEL_RADIUS (70/2) // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#define WHEEL_PERIMETER (2*PI*WHEEL_RADIUS) // 220mm // 2*PI*r // r=7/2=3.5
#define MOTOR_TOTAL_STEPS 200 // Total de steps do motor para dar uma volta
#define MOTOR_MICROSTEPS 8 // Quantos steps tem que dar para correr um step real

#define MOTORS_MAX_SPEED (160.0 * MOTOR_MICROSTEPS) // Ano passado 150
//#define MOTORS_MAX_SPEED (500.0 * MOTOR_MICROSTEPS) // Ano passado 150
#define MOTORS_ACELERATION (3000.0 * MOTOR_MICROSTEPS) // Ano passado 3000
// #define MOTORS_ACELERATION (1000.0 * MOTOR_MICROSTEPS) // Ano passado 3000

//const double PI = 3.141592;
//#define PI 3.141592
//const unsigned long TICK_MAIN_PROCESS = 50; //t(ms)

//#define STEPS_MM (WHEEL_PERIMETER / (MOTOR_TOTAL_STEPS * MOTOR_MICROSTEPS)) // Numero de steps por mm
#define STEPS_MM ((MOTOR_TOTAL_STEPS * MOTOR_MICROSTEPS) / WHEEL_PERIMETER) // Numero de steps por mm

#define PING_MAX_DISTANCE 300 // Em mm (será convertido para cm no main)

//#define THRESHOLD_FLOOR_SENSOR 300
#define THRESHOLD_FLOOR_SENSOR 150

#define NONE 0
#define LEFT 1
#define RIGHT 2

// SERVO CALIBRATION VALUES ////////////////////////////////////////////////////////////////////////////////////////////
// Values in degrees
//#define MOTOR_SERVO_0   18 // 15
//#define MOTOR_SERVO_180 177 // 175

// Values in time units
#define MOTOR_SERVO_0 729
#define MOTOR_SERVO_180 (2369 + 1)

// BEACON THRESHOLDS ///////////////////////////////////////////////////////////////////////////////////////////////////
#define BEACON_MIN_FREQ_THRESHOLD  550
#define BEACON_MAX_FREQ_THRESHOLD  650
#define BEACON_MIN_PWM_THRESHOLD   0.25
#define BEACON_MAX_PWM_THRESHOLD   0.35

#endif //PACMAN_CONSTANTS_H

