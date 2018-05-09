//#include <Arduino.h>
//#include <AccelStepper.h>
//
//#define PIN_MOTOR_DIR 8  // Motor Direção
//#define PIN_MOTOR_STEP 9    // Motor passo
//
//#define MIN_DELAY_MOTORS 17000 // Microseconds
//#define MOTOR_TOTAL_STEPS 200 // Microseconds
//#define MOTOR_MICROSTEPS 8 // Microseconds
//
//AccelStepper stepper(AccelStepper::DRIVER, PIN_MOTOR_STEP, PIN_MOTOR_DIR);
////AccelStepper(PIN_MOTOR_STEP, uint8_t pin1 = 2, uint8_t pin2 = 3, uint8_t pin3 = 4, uint8_t pin4 = 5);
////AccelStepper stepper(2, 10, 11);
//
//int pos = 3600;
//
//void setup() {
//    stepper.setMaxSpeed(100.0 * MOTOR_MICROSTEPS);
//    stepper.setAcceleration(100.0 * MOTOR_MICROSTEPS);
//
////    stepper.setMaxSpeed(300);
////    stepper.setAcceleration(100);
//
////    stepper.setSpeed(50);
//}
//
//void loop() {
////    stepper.runSpeed();
////    stepper.run();
//
//    if (stepper.distanceToGo() == 0) {
//        delay(500);
//        pos = -pos;
//        stepper.moveTo(pos);
//    }
//    stepper.run();
//}


//
///*
// * Using accelerated motion ("linear speed") in nonblocking mode
// *
// * Copyright (C)2015-2017 Laurentiu Badea
// *
// * This file may be redistributed under the terms of the MIT license.
// * A copy of this license has been included with this distribution in the file LICENSE.
// */
//#include <Arduino.h>
//
//// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
//#define MOTOR_STEPS 200
//// Target RPM for cruise speed
//#define RPM 120
//// Acceleration and deceleration values are always in FULL steps / s^2
//#define MOTOR_ACCEL 2000
//#define MOTOR_DECEL 1000
//
//// Microstepping mode. If you hardwired it to save pins, set to the same value here.
//#define MICROSTEPS 16
//
//#define DIR 8
//#define STEP 9
//#define ENABLE 13 // optional (just delete ENABLE from everywhere if not used)
//
///*
// * Choose one of the sections below that match your board
// */
//
////#include "DRV8834.h"
////#define M0 10
////#define M1 11
////DRV8834 stepper(MOTOR_STEPS, DIR, STEP, ENABLE, M0, M1);
//
//// #include "A4988.h"
//// #define MS1 10
//// #define MS2 11
//// #define MS3 12
//// A4988 stepper(MOTOR_STEPS, DIR, STEP, ENABLE, MS1, MS2, MS3);
//
//#include "DRV8825.h"
//#define MODE0 10
//#define MODE1 11
//#define MODE2 12
//DRV8825 stepper(MOTOR_STEPS, DIR, STEP, ENABLE, MODE0, MODE1, MODE2);
//
//// #include "DRV8880.h"
//// #define M0 10
//// #define M1 11
//// #define TRQ0 6
//// #define TRQ1 7
//// DRV8880 stepper(MOTOR_STEPS, DIR, STEP, ENABLE, M0, M1, TRQ0, TRQ1);
//
//// #include "BasicStepperDriver.h" // generic
//// BasicStepperDriver stepper(DIR, STEP);
//
//void setup() {
//    Serial.begin(115200);
//
//    stepper.begin(RPM, MICROSTEPS);
//    stepper.enable();
//    // set current level (for DRV8880 only). Valid percent values are 25, 50, 75 or 100.
//    // stepper.setCurrent(100);
//
//    /*
//     * Set LINEAR_SPEED (accelerated) profile.
//     */
//    stepper.setSpeedProfile(stepper.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);
//
//    Serial.println("START");
//    /*
//     * Using non-blocking mode to print out the step intervals.
//     * We could have just as easily replace everything below this line with
//     * stepper.rotate(360);
//     */
//    stepper.startRotate(360);
//}
//
//void loop() {
//    static int step = 0;
//    unsigned wait_time = stepper.nextAction();
//    delay(100);
//    if (wait_time){
//        Serial.print("  step="); Serial.print(step++);
//        Serial.print("  dt="); Serial.print(wait_time);
//        Serial.print("  rpm="); Serial.print(stepper.getCurrentRPM());
//        Serial.println();
//    } else {
//        stepper.disable();
//        Serial.println("END");
//        delay(3600000);
//    }
//}
//

///*
// * Simple demo, should work with any driver board
// *
// * Connect STEP, DIR as indicated
// *
// * Copyright (C)2015-2017 Laurentiu Badea
// *
// * This file may be redistributed under the terms of the MIT license.
// * A copy of this license has been included with this distribution in the file LICENSE.
// */
//#include <Arduino.h>
//#include "BasicStepperDriver.h"
//
//// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
//#define MOTOR_STEPS 200
//#define RPM (120 / 2)
//
//// Since microstepping is set externally, make sure this matches the selected mode
//// If it doesn't, the motor will move at a different RPM than chosen
//// 1=full step, 2=half step etc.
//#define MICROSTEPS 2
//
//// All the wires needed for full functionality
//#define DIR 8
//#define STEP 9
////Uncomment line to use enable/disable functionality
////#define ENABLE 13
//
//// 2-wire basic config, microstepping is hardwired on the driver
//BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP);
//
////Uncomment line to use enable/disable functionality
////BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP, ENABLE);
//
//void setup() {
//    stepper.begin(RPM, MICROSTEPS);
//}
//
//void loop() {
//
//    // energize coils - the motor will hold position
//    // stepper.enable();
//
//    /*
//     * Moving motor one full revolution using the degree notation
//     */
//    stepper.rotate(360);
//
//    /*
//     * Moving motor to original position using steps
//     */
//    stepper.move(-MOTOR_STEPS*MICROSTEPS);
//
//    // pause and allow the motor to be moved by hand
//    // stepper.disable();
//
//    delay(5000);
//}


/*
#include <Arduino.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Mouse.h>

//#if defined(ARDUINO_AVR_NANO) || defined(ARDUINO_AVR_ADK)
//
//#else
#define Serial SerialUSB

//#endif

MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;
int vx, vy;
int button1 = 6;
int button2 = 7;
int buttonState1 = 0;
int buttonState2 = 0;


void setup() {
    Serial.begin(115200);

    while(!Serial);

    Serial.println("Setup");
    Wire.begin();
    pinMode(button1, INPUT);
    pinMode(button2, INPUT);
    Serial.println("Done");
    mpu.initialize();
    if (!mpu.testConnection()) { while (1); }
    Serial.println("Done");
}

void loop() {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    vx = (gx+15)/150;
    vy = -(gz-100)/150;

    Serial.print("gx = ");
    Serial.print(gx);
    Serial.print(" | gz = ");
    Serial.print(gz);

    Serial.print("        | X = ");
    Serial.print(vx);
    Serial.print(" | Y = ");
    Serial.println(vy);

    Mouse.move(vx, vy);

    buttonState1 = digitalRead(button1);
    buttonState2 = digitalRead(button2);
//
//    if (buttonState1 == HIGH) {
//        Mouse.press(MOUSE_LEFT);
//        delay(100);
//        Mouse.release(MOUSE_LEFT);
//        delay(200);
//    }
//    else if(buttonState2 == HIGH) {
//        Mouse.press(MOUSE_RIGHT);
//        delay(100);
//        Mouse.release(MOUSE_RIGHT);
//        delay(200);
//    }
    delay(80);
}
*/


//#include <Arduino.h>
//#define Serial SerialUSB
//
//// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
//// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
//// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
////
//// Changelog:
////      2013-05-08 - added seamless Fastwire support
////                 - added note about gyro calibration
////      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
////      2012-06-20 - improved FIFO overflow handling and simplified read process
////      2012-06-19 - completely rearranged DMP initialization code and simplification
////      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
////      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
////      2012-06-05 - add gravity-compensated initial reference frame acceleration output
////                 - add 3D math helper file to DMP6 example sketch
////                 - add Euler output and Yaw/Pitch/Roll output formats
////      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
////      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
////      2012-05-30 - basic DMP initialization working
//
///* ============================================
//I2Cdev device library code is placed under the MIT license
//Copyright (c) 2012 Jeff Rowberg
//
//Permission is hereby granted, free of charge, to any person obtaining a copy
//of this software and associated documentation files (the "Software"), to deal
//in the Software without restriction, including without limitation the rights
//to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//copies of the Software, and to permit persons to whom the Software is
//furnished to do so, subject to the following conditions:
//
//The above copyright notice and this permission notice shall be included in
//all copies or substantial portions of the Software.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.
//===============================================
//*/
//
//// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
//// for both classes must be in the include path of your project
//#include "I2Cdev.h"
//
//#include "MPU6050_6Axis_MotionApps20.h"
////#include "MPU6050.h" // not necessary if using MotionApps include file
//
//// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
//// is used in I2Cdev.h
//#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//#include "Wire.h"
//#endif
//
//// class default I2C address is 0x68
//// specific I2C addresses may be passed as a parameter here
//// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
//// AD0 high = 0x69
//MPU6050 mpu;
////MPU6050 mpu(0x69); // <-- use for AD0 high
//
///* =========================================================================
//   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
//   depends on the MPU-6050's INT pin being connected to the Arduino's
//   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
//   digital I/O pin 2.
// * ========================================================================= */
//
///* =========================================================================
//   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
//   when using Serial.write(buf, len). The Teapot output uses this method.
//   The solution requires a modification to the Arduino USBAPI.h file, which
//   is fortunately simple, but annoying. This will be fixed in the next IDE
//   release. For more info, see these links:
//
//   http://arduino.cc/forum/index.php/topic,109987.0.html
//   http://code.google.com/p/arduino/issues/detail?id=958
// * ========================================================================= */
//
//
//
//// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
//// quaternion components in a [w, x, y, z] format (not best for parsing
//// on a remote host such as Processing or something though)
////#define OUTPUT_READABLE_QUATERNION
//
//// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
//// (in degrees) calculated from the quaternions coming from the FIFO.
//// Note that Euler angles suffer from gimbal lock (for more info, see
//// http://en.wikipedia.org/wiki/Gimbal_lock)
////#define OUTPUT_READABLE_EULER
//
//// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
//// pitch/roll angles (in degrees) calculated from the quaternions coming
//// from the FIFO. Note this also requires gravity vector calculations.
//// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
//// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
////#define OUTPUT_READABLE_YAWPITCHROLL
//
//// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
//// components with gravity removed. This acceleration reference frame is
//// not compensated for orientation, so +X is always +X according to the
//// sensor, just without the effects of gravity. If you want acceleration
//// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
////#define OUTPUT_READABLE_REALACCEL
//
//// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
//// components with gravity removed and adjusted for the world frame of
//// reference (yaw is relative to initial orientation, since no magnetometer
//// is present in this case). Could be quite handy in some cases.
////#define OUTPUT_READABLE_WORLDACCEL
//
//// uncomment "OUTPUT_TEAPOT" if you want output that matches the
//// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT
//
//
//
//#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
//#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
//bool blinkState = false;
//
//// MPU control/status vars
//bool dmpReady = false;  // set true if DMP init was successful
//uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
//uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
//uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
//uint16_t fifoCount;     // count of all bytes currently in FIFO
//uint8_t fifoBuffer[64]; // FIFO storage buffer
//
//// orientation/motion vars
//Quaternion q;           // [w, x, y, z]         quaternion container
//VectorInt16 aa;         // [x, y, z]            accel sensor measurements
//VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
//VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
//VectorFloat gravity;    // [x, y, z]            gravity vector
//float euler[3];         // [psi, theta, phi]    Euler angle container
//float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
//
//// packet structure for InvenSense teapot demo
//uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
//
//
//
//// ================================================================
//// ===               INTERRUPT DETECTION ROUTINE                ===
//// ================================================================
//
//volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
//void dmpDataReady() {
//    mpuInterrupt = true;
//}
//
//
//
//// ================================================================
//// ===                      INITIAL SETUP                       ===
//// ================================================================
//
//void setup() {
//    // join I2C bus (I2Cdev library doesn't do this automatically)
//#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//    Wire.begin();
//    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
//#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
//    Fastwire::setup(400, true);
//#endif
//
//    // initialize serial communication
//    // (115200 chosen because it is required for Teapot Demo output, but it's
//    // really up to you depending on your project)
//    Serial.begin(115200);
//    while (!Serial); // wait for Leonardo enumeration, others continue immediately
//
//    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
//    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
//    // the baud timing being too misaligned with processor ticks. You must use
//    // 38400 or slower in these cases, or use some kind of external separate
//    // crystal solution for the UART timer.
//
//    // initialize device
//    Serial.println(F("Initializing I2C devices..."));
//    mpu.initialize();
//    pinMode(INTERRUPT_PIN, INPUT);
//
//    // verify connection
//    Serial.println(F("Testing device connections..."));
//    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
//
//    // wait for ready
//    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
//    while (Serial.available() && Serial.read()); // empty buffer
//    while (!Serial.available());                 // wait for data
//    while (Serial.available() && Serial.read()); // empty buffer again
//
//    // load and configure the DMP
//    Serial.println(F("Initializing DMP..."));
//    devStatus = mpu.dmpInitialize();
//
//    // supply your own gyro offsets here, scaled for min sensitivity
//    mpu.setXGyroOffset(220);
//    mpu.setYGyroOffset(76);
//    mpu.setZGyroOffset(-85);
//    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
//
//    // make sure it worked (returns 0 if so)
//    if (devStatus == 0) {
//        // turn on the DMP, now that it's ready
//        Serial.println(F("Enabling DMP..."));
//        mpu.setDMPEnabled(true);
//
//        // enable Arduino interrupt detection
//        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
//        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
//        mpuIntStatus = mpu.getIntStatus();
//
//        // set our DMP Ready flag so the main loop() function knows it's okay to use it
//        Serial.println(F("DMP ready! Waiting for first interrupt..."));
//        dmpReady = true;
//
//        // get expected DMP packet size for later comparison
//        packetSize = mpu.dmpGetFIFOPacketSize();
//    } else {
//        // ERROR!
//        // 1 = initial memory load failed
//        // 2 = DMP configuration updates failed
//        // (if it's going to break, usually the code will be 1)
//        Serial.print(F("DMP Initialization failed (code "));
//        Serial.print(devStatus);
//        Serial.println(F(")"));
//    }
//
//    // configure LED for output
//    pinMode(LED_PIN, OUTPUT);
//}
//
//
//
//// ================================================================
//// ===                    MAIN PROGRAM LOOP                     ===
//// ================================================================
//
//void loop() {
//    // if programming failed, don't try to do anything
//    if (!dmpReady) return;
//
//    // wait for MPU interrupt or extra packet(s) available
//    while (!mpuInterrupt && fifoCount < packetSize) {
//        // other program behavior stuff here
//        // .
//        // .
//        // .
//        // if you are really paranoid you can frequently test in between other
//        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
//        // while() loop to immediately process the MPU data
//        // .
//        // .
//        // .
//    }
//
//    // reset interrupt flag and get INT_STATUS byte
//    mpuInterrupt = false;
//    mpuIntStatus = mpu.getIntStatus();
//
//    // get current FIFO count
//    fifoCount = mpu.getFIFOCount();
//
//    // check for overflow (this should never happen unless our code is too inefficient)
//    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
//        // reset so we can continue cleanly
//        mpu.resetFIFO();
//        Serial.println(F("FIFO overflow!"));
//
//        // otherwise, check for DMP data ready interrupt (this should happen frequently)
//    } else if (mpuIntStatus & 0x02) {
//        // wait for correct available data length, should be a VERY short wait
//        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
//
//        // read a packet from FIFO
//        mpu.getFIFOBytes(fifoBuffer, packetSize);
//
//        // track FIFO count here in case there is > 1 packet available
//        // (this lets us immediately read more without waiting for an interrupt)
//        fifoCount -= packetSize;
//
//#ifdef OUTPUT_READABLE_QUATERNION
//        // display quaternion values in easy matrix form: w x y z
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            Serial.print("quat\t");
//            Serial.print(q.w);
//            Serial.print("\t");
//            Serial.print(q.x);
//            Serial.print("\t");
//            Serial.print(q.y);
//            Serial.print("\t");
//            Serial.println(q.z);
//#endif
//
//#ifdef OUTPUT_READABLE_EULER
//        // display Euler angles in degrees
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            mpu.dmpGetEuler(euler, &q);
//            Serial.print("euler\t");
//            Serial.print(euler[0] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(euler[1] * 180/M_PI);
//            Serial.print("\t");
//            Serial.println(euler[2] * 180/M_PI);
//#endif
//
//#ifdef OUTPUT_READABLE_YAWPITCHROLL
//        // display Euler angles in degrees
//        mpu.dmpGetQuaternion(&q, fifoBuffer);
//        mpu.dmpGetGravity(&gravity, &q);
//        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//        Serial.print("ypr\t");
//        Serial.print(ypr[0] * 180/M_PI);
//        Serial.print("\t");
//        Serial.print(ypr[1] * 180/M_PI);
//        Serial.print("\t");
//        Serial.println(ypr[2] * 180/M_PI);
//#endif
//
//#ifdef OUTPUT_READABLE_REALACCEL
//        // display real acceleration, adjusted to remove gravity
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            mpu.dmpGetAccel(&aa, fifoBuffer);
//            mpu.dmpGetGravity(&gravity, &q);
//            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//            Serial.print("areal\t");
//            Serial.print(aaReal.x);
//            Serial.print("\t");
//            Serial.print(aaReal.y);
//            Serial.print("\t");
//            Serial.println(aaReal.z);
//#endif
//
//#ifdef OUTPUT_READABLE_WORLDACCEL
//        // display initial world-frame acceleration, adjusted to remove gravity
//            // and rotated based on known orientation from quaternion
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            mpu.dmpGetAccel(&aa, fifoBuffer);
//            mpu.dmpGetGravity(&gravity, &q);
//            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
//            Serial.print("aworld\t");
//            Serial.print(aaWorld.x);
//            Serial.print("\t");
//            Serial.print(aaWorld.y);
//            Serial.print("\t");
//            Serial.println(aaWorld.z);
//#endif
//
//#ifdef OUTPUT_TEAPOT
//        // display quaternion values in InvenSense Teapot demo format:
//            teapotPacket[2] = fifoBuffer[0];
//            teapotPacket[3] = fifoBuffer[1];
//            teapotPacket[4] = fifoBuffer[4];
//            teapotPacket[5] = fifoBuffer[5];
//            teapotPacket[6] = fifoBuffer[8];
//            teapotPacket[7] = fifoBuffer[9];
//            teapotPacket[8] = fifoBuffer[12];
//            teapotPacket[9] = fifoBuffer[13];
//            Serial.write(teapotPacket, 14);
//            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
//#endif
//
//        // blink LED to indicate activity
//        blinkState = !blinkState;
//        digitalWrite(LED_PIN, blinkState);
//    }
//}

#include <Arduino.h>
/*
 Mouse Controller Example

 Shows the output of a USB Mouse connected to
 the Native USB port on an Arduino Due Board.

 created 8 Oct 2012
 by Cristian Maglie

 http://arduino.cc/en/Tutorial/MouseController

 This samlple code is part of the public domain.
 */

// Require mouse control library
#include <MouseController.h>

// Initialize USB Controller
USBHost usb;

// Attach mouse controller to USB
MouseController mouse(usb);

// variables for mouse button states
boolean leftButton = false;
boolean middleButton = false;
boolean rightButton = false;

// This function intercepts mouse movements
void mouseMoved() {
    Serial.print("Move: ");
    Serial.print(mouse.getXChange());
    Serial.print(", ");
    Serial.println(mouse.getYChange());
}

// This function intercepts mouse movements while a button is pressed
void mouseDragged() {
    Serial.print("DRAG: ");
    Serial.print(mouse.getXChange());
    Serial.print(", ");
    Serial.println(mouse.getYChange());
}

// This function intercepts mouse button press
void mousePressed() {
    Serial.print("Pressed: ");
    if (mouse.getButton(LEFT_BUTTON)){
        Serial.print("L");
        leftButton = true;
    }
    if (mouse.getButton(MIDDLE_BUTTON)){
        Serial.print("M");
        middleButton = true;
    }
    if (mouse.getButton(RIGHT_BUTTON)){
        Serial.print("R");
        Serial.println();
        rightButton = true;
    }
}

// This function intercepts mouse button release
void mouseReleased() {
    Serial.print("Released: ");
    if (!mouse.getButton(LEFT_BUTTON) && left==true) {
        Serial.print("L");
        leftButton = false;
    }
    if (!mouse.getButton(MIDDLE_BUTTON) && middle==true) {
        Serial.print("M");
        middleButton = false;
    }
    if (!mouse.getButton(RIGHT_BUTTON) && right==true) {
        Serial.print("R");
        rightButton = false;
    }
    Serial.println();
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Program started");
    delay(200);
}

void loop()
{
    // Process USB tasks
    usb.Task();
}