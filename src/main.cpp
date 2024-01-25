#include <Arduino.h>
#include "Motor.h"
#include "QTR.h"

/* Figures */
const uint8_t figuresCount = 0;
const uint8_t figures[figuresCount] = {};

/* Motor settings */
// L298N (Motor pins)
const uint8_t enL = 3;  // PWM output to set the motor speed
const uint8_t fwdL = 5; // if HIGH motor is going forward
const uint8_t bwdL = 7; // if HIGH motor is going backward

const uint8_t enR = 4;  // PWM output to set the motor speed
const uint8_t fwdR = 6; // if HIGH motor is going forward
const uint8_t bwdR = 8; // if HIGH motor is going backward
// Setup the motors
Motor motorL;
Motor motorR;
// Motor speed limits
const int16_t motorMinSpeed = -255;
const int16_t motorMaxSpeed = 255;
const uint8_t baseSpeed = 100;
// Motor
int16_t motorSpeedLeft;
int16_t motorSpeedRight;

/* Line sensor settings */
// QTR-8RC (Line sensor pins)
const uint8_t pinsQTR[8] = {15, 16, 17, 18, 19, 20, 21, 22}; // Analog pins// 0: left ; 7: right
const uint8_t enQTR = 14;                                    // enable the output of all the leds
// Setup the line sensor
QTR qtr;
// Line sensor variables
int16_t lineValues[4];

// Inlude the figures after all the declarations
#include "Figures.h"

/* PID settings */
// PID parameters
const double Kp = 25; // Proportional gain
const double Ki = 5; // Integral gain
const double Kd = 10; // Derivative gain
// Setpoint - Desired position of the robot on the line
const uint8_t setpoint = 0;
// Variables for PID
double pid;
double error;
double errorPrev;
double proportional;
double integral;
double integralPrev;
double derivative;

/* Loop rate settings */
// Loop rate variables
float dt;
uint32_t currentTime, previousTime, elapsedTime = 0;
uint32_t printCounter, serialCounter;
uint32_t blinkCounter, blinkDelay;
bool blinkAlternate;

/* Functions signatures definitons */
void setInOut();

void acquireLine();
void calculateError();
void calculatePID();
void commandMotors();

void followLine();

void loopBlink();
void loopRate(int freq);
void printLoopRate();

void setup()
{
    Serial.begin(115200); // Init the Serial communication

    setInOut(); // Define every pinMode() and constructors

    // Init the motors speed at 0
    motorL.stop();
    motorR.stop();

    qtr.calibrate(2000); // Calibrate the QTR sensor for ~2sec

    // Set the initial speed of the robot
    motorL.setSpeed(baseSpeed);
    motorR.setSpeed(baseSpeed);
}

void loop()
{
    previousTime = currentTime;
    currentTime = micros();
    dt = (currentTime - previousTime) / 1000000.0;

    /* ----- Main part of the loop ----- */
    followLine();
    /* --------------------------------- */

    loopBlink(); // Indicate we are in main loop with short blink every 1.5 seconds
    loopRate(2000); // Set the loop speed at 2000Hz
}

void setInOut()
{
    // Led builtin
    pinMode(LED_BUILTIN, OUTPUT);

    // L298N
    motorL.init(enL, fwdL, bwdL);
    motorR.init(enR, fwdR, bwdR);

    // QTR-8RC
    qtr.init(enQTR, pinsQTR[0], pinsQTR[1], pinsQTR[2], pinsQTR[3], pinsQTR[4], pinsQTR[5], pinsQTR[6], pinsQTR[7]);
}

void acquireLine()
{
    // 4 sensors in the middle
    lineValues[0] = qtr.getValue(2);
    lineValues[1] = qtr.getValue(3);
    lineValues[2] = qtr.getValue(4);
    lineValues[3] = qtr.getValue(5);
}

void calculateError()
{
    acquireLine();

    int16_t position = 0;
    for (int i = 0; i < 4; i++)
    {
        // Weighted sum of sensor values to calculate the position
        position += (i - 1.5) * lineValues[i];
    }
    error = position / 100; // Scaling factor for better PID performance
}

void calculatePID()
{
    // Calculate PID components
    proportional = error;
    integral = integralPrev + error * dt;
    derivative = (error - errorPrev) / dt;

    pid = Kp * proportional + Ki * integral + Kd * derivative;

    // Calculate motor speeds
    motorSpeedLeft = constrain(baseSpeed + pid, motorMinSpeed, motorMaxSpeed);
    motorSpeedRight = constrain(baseSpeed - pid, motorMinSpeed, motorMaxSpeed);

    // Save the current error and integral for the next iteration
    errorPrev = error;
    integralPrev = integral;
}

void commandMotors()
{
    motorL.setSpeed(motorSpeedLeft);
    motorR.setSpeed(motorSpeedRight);
}

void followLine()
{
    calculateError(); // Calculate error (difference between actual position and setpoint)

    calculatePID(); // Calculate the new movement of the robot based on the error

    commandMotors(); // Send the values calculated by the PID to the motors
}

void loopBlink()
{
    if (currentTime - blinkCounter > blinkDelay)
    {
        blinkCounter = micros();
        digitalWrite(LED_BUILTIN, blinkAlternate);

        if (blinkAlternate == 1)
        {
            blinkAlternate = 0;
            blinkDelay = 100000;
        }
        else if (blinkAlternate == 0)
        {
            blinkAlternate = 1;
            blinkDelay = 2000000;
        }
    }
}

void loopRate(int freq)
{
    float invFreq = 1.0 / freq * 1000000.0;
    uint32_t checker = micros();

    // Sit in loop until appropriate time has passed
    while (invFreq > (checker - currentTime))
    {
        checker = micros();
    }
}

void printLoopRate()
{
    if (currentTime - printCounter > 10000)
    {
        printCounter = micros();
        
        Serial.print(F("dt = "));
        Serial.println(dt * 1000000.0);
    }
}

void printMotorSpeed()
{
    if (currentTime - printCounter > 10000)
    {
        printCounter = micros();
        
        Serial.print(F("Motor L: "));
        Serial.print(motorSpeedLeft);
        Serial.print(F("Motor R: "));
        Serial.print(motorSpeedRight);
    }
}
