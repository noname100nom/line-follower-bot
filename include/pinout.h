#include <Arduino.h>

// L298N
const uint8_t enL = 3; // PWM output to set the motor speed
const uint8_t fwdL = 5; // if HIGH motor is going forward
const uint8_t bwdL = 7; // if HIGH motor is going backward

const uint8_t enR = 4; // PWM output to set the motor speed
const uint8_t fwdR = 6; // if HIGH motor is going forward
const uint8_t bwdR = 8; // if HIGH motor is going backward

// QTR-8RC
const uint8_t pinLine[8] = {14, 15, 16, 17, 18, 19, 20, 21}; // Analog pins// 0: left ; 7: right
const uint8_t enLine = 22; // enable the output of all the leds

void setInOut()
{
    // L298N
    pinMode(enL, OUTPUT);
    pinMode(fwdL, OUTPUT);
    pinMode(bwdL, OUTPUT);

    pinMode(enR, OUTPUT);
    pinMode(fwdR, OUTPUT);
    pinMode(bwdR, OUTPUT);

    // QTR-8RC
    pinMode(pinLine[0], INPUT);
    pinMode(pinLine[1], INPUT);
    pinMode(pinLine[2], INPUT);
    pinMode(pinLine[3], INPUT);
    pinMode(pinLine[4], INPUT);
    pinMode(pinLine[5], INPUT);
    pinMode(pinLine[6], INPUT);
    pinMode(pinLine[7], INPUT);

    pinMode(enLine, OUTPUT);
    digitalWrite(enLine, HIGH); // Enable the line leds

    // Touch button
    pinMode(0, INPUT);
}
