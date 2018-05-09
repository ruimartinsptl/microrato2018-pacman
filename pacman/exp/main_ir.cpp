
//#define IR_PIN_INT

#include <Arduino.h>

#ifndef IR_PIN_INT
#include <IRremote.h>
#endif

#ifdef ARDUINO_AVR_NANO
#define IR_PIN 2
#else
#define Serial SerialUSB
#define IR_PIN 22
#endif

#ifdef IR_PIN_INT
void rising();
void falling();

volatile uint32_t pwm_value_on = 0;
volatile uint32_t pwm_value_off = 0;
volatile uint32_t prev_time_on = 0;
volatile uint32_t prev_time_off = 0;

#endif

#ifndef IR_PIN_INT

IRrecv irrecv(IR_PIN);
decode_results results;

#endif

// the setup function runs once when you press reset or power the board
void setup() {
    Serial.begin(115200);

    while(!Serial);

#ifndef IR_PIN_INT
    Serial.println("Enabling IRin");
    irrecv.enableIRIn(); // Start the receiver
    Serial.println("Enabled IRin");
#endif

    // initialize digital pin LED_BUILTIN as an output.
    pinMode(LED_BUILTIN, OUTPUT);

#ifdef IR_PIN_INT
    pinMode(IR_PIN, INPUT);
    // when pin D22 goes high, call the rising function
    attachInterrupt(IR_PIN, rising, RISING);
//    attachInterrupt(IR_PIN, falling, FALLING);
#endif
}

// the loop function runs over and over again forever
void loop() {
#ifndef IR_PIN_INT
    if (irrecv.decode(&results)) {
        Serial.println(results.value, HEX);
        irrecv.resume(); // Receive the next value
    }
    delay(100);
#endif

#ifdef IR_PIN_INT
//    Serial.print("*");
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(250);                       // wait for a second
//    Serial.print(".");
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(250);                       // wait for a second
#endif
}

#ifdef IR_PIN_INT

void rising() {
    attachInterrupt(IR_PIN, falling, FALLING);
    prev_time_on = micros();

    pwm_value_off = micros() - prev_time_off;

    Serial.print("-");
    Serial.println(pwm_value_off);
}

void falling() {
    attachInterrupt(IR_PIN, rising, RISING);
    prev_time_off = micros();

    pwm_value_on = micros() - prev_time_on;

    Serial.print("+");
    Serial.println(pwm_value_on);
}

#endif