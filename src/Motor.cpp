// Motor.cpp

#include <Arduino.h>
#include "Motor.h"

Motor::Motor()
{
    // Default constructor
}

void Motor::init(uint8_t en, uint8_t fwd, uint8_t bwd)
{
    // Set the pins for the motor
    this->en = en;
    this->fwd = fwd;
    this->bwd = bwd;

    // Configure pins as OUTPUT
    pinMode(en, OUTPUT);
    pinMode(fwd, OUTPUT);
    pinMode(bwd, OUTPUT);
}

void Motor::setSpeed(int16_t speed)
{
    // Ensure the speed is within the valid range (-255 to 255)
    speed = constrain(speed, -255, 255);

    if (speed == 0) // Stop
    {
        stop(); // Call the stop function to stop the motors
    }
    else if (speed > 0) // Forward
    {
        // Set the motor in forward mode
        digitalWrite(fwd, HIGH);
        digitalWrite(bwd, LOW);

        // Set the motor speed using analogWrite
        analogWrite(en, speed);
    }
    else if (speed < 0) // Backward
    {
        // Set the motor in backward mode
        digitalWrite(fwd, LOW);
        digitalWrite(bwd, HIGH);

        // Set the inverted motor speed using analogWrite
        analogWrite(en, speed * -1);
    }
}

void Motor::stop()
{
    digitalWrite(fwd, LOW);
    digitalWrite(bwd, LOW);

    analogWrite(en, LOW);
}
