#include <Arduino.h>

// L298N
const uint8_t enL = 3; // PWM output to set the motor speed
const uint8_t fwdL = 5; // if HIGH motor is going forward
const uint8_t bwdL = 7; // if HIGH motor is going backward

const uint8_t enR = 4; // PWM output to set the motor speed
const uint8_t fwdR = 6; // if HIGH motor is going forward
const uint8_t bwdR = 8; // if HIGH motor is going backward

// QTR-8RC
const uint8_t pinsQTR[8] = {14, 15, 16, 17, 18, 19, 20, 21}; // Analog pins// 0: left ; 7: right
const uint8_t enQTR = 22; // enable the output of all the leds

void setInOut()
{
    // Led builtin
    pinMode(LED_BUILTIN, OUTPUT);

    // L298N
    motorL.init(enL, fwdL, bwdL);
    motorL.init(enR, fwdR, bwdR);

    // QTR-8RC
    qtr.init(enQTR, pinsQTR[0], pinsQTR[1], pinsQTR[2], pinsQTR[3], pinsQTR[4], pinsQTR[5], pinsQTR[6], pinsQTR[7]);
}
