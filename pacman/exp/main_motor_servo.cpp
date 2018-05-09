#include <Arduino.h>
#include <Servo.h>

#define Serial SerialUSB



//#define PIN_WHITE_OUTPUT_LED 2
//#define PIN_FLOOR_SENSOR A5

#define SERVO_STEP_ANGLE 8
#define LEFT 1
#define RIGHT 2

#define PIN_MOTOR_SERVO 2

Servo motor_servo_farol;
int motor_servo_farol_pos = 0;
byte motor_servo_farol_dir = LEFT;
bool servo_motor_is_on = false;


// SERVO MOTOR /////////////////////////////////////////////////////////////////////////////////////////////////////////
void turn_on_servo(){
    motor_servo_farol.attach(PIN_MOTOR_SERVO);
    servo_motor_is_on = true;
}

void turn_off_servo(){
    motor_servo_farol.detach();
    servo_motor_is_on = false;
}

void set_servo_degree(int degree){
//void set_servo_degree(int degree, bool move_fast){
    motor_servo_farol.write(degree);
//    if (move_fast){
//        motor_servo_farol.write(degree);
//    }
//    else{
//        if (motor_servo_farol_pos > degree){
//            for (int pos = motor_servo_farol_pos; pos >= degree; pos -= 1) {
//                motor_servo_farol.write(pos);  // tell servo to go to position in variable "pos"
//                delay(15);  // waits 15ms for the servo to reach the position
//            }
//            motor_servo_farol_pos = degree;
//        } else if (motor_servo_farol_pos < degree){
//            for (int pos = motor_servo_farol_pos; pos <= degree; pos += 1) {
//                motor_servo_farol.write(pos);  // tell servo to go to position in variable "pos"
//                delay(15);  // waits 15ms for the servo to reach the position
//            }
//            motor_servo_farol_pos = degree;
//        }
//    };
}

void servo_rotate(){
    if (servo_motor_is_on) {
        if (motor_servo_farol_dir == RIGHT) {
            motor_servo_farol_pos+=SERVO_STEP_ANGLE;
        } else {
            motor_servo_farol_pos-=SERVO_STEP_ANGLE;
        }
        if (motor_servo_farol_pos >= 170) {
            motor_servo_farol_pos = 170;
            motor_servo_farol_dir = LEFT;
        }
        if (motor_servo_farol_pos <= 10) {
            motor_servo_farol_pos = 10;
            motor_servo_farol_dir = RIGHT;
        }
        set_servo_degree(motor_servo_farol_pos);
    }
}
// SERVO MOTOR /////////////////////////////////////////////////////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void setup() {

    Serial.begin(115200);
    while(!Serial); // Obrigatorio para o Arduino DUE

    // initialize digital pin LED_BUILTIN as an output.
    pinMode(LED_BUILTIN, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
    turn_on_servo();
    Serial.println("0");
    set_servo_degree(0);
    Serial.println("0 OK");
    Serial.println("180");
    set_servo_degree(180);
    Serial.println("180 OK");
//    set_servo_degree(0);
    Serial.println("90");
    set_servo_degree(90);
    Serial.println("90 OK");
    turn_off_servo();

    delay(10);
}