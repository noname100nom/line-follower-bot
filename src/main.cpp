#include <Arduino.h>
#include "pinout.h"
#include "Motor/Motor.h"
#include "QTR/QTR.h"
#include "Figures/Figures.h"

/* Figures */
const uint8_t figuresCount = 0;
const uint8_t figures[figuresCount] = {};

/* Motor settings */
// Setup the motors
Motor motorL;
Motor motorR;
// Motor speed limits
const int16_t motorMinSpeed = -255;
const int16_t motorMaxSpeed = 255;
// Motor
int16_t motorSpeedLeft;
int16_t motorSpeedRight;

/* Line sensor settings */
// Setup the line sensor
QTR qtr;
// Line sensor variables
int16_t lineValues[4];

/* PID settings */
// PID parameters
const double Kp = 2.0; // Proportional gain
const double Ki = 0.1; // Integral gain
const double Kd = 0.5; // Derivative gain
// Setpoint - Desired position of the robot on the line
const int16_t setpoint = 0;
// Variables for PID
double error = 0;
double lastError = 0;
double integral = 0;

/* Loop rate settings */
// Loop rate variables
float dt;
uint32_t currentTime, previousTime, elapsedTime = 0;
uint32_t printCounter, serialCounter;
uint32_t blinkCounter, blinkDelay;
bool blinkAlternate;

/* Functions signatures definitons */
void acquireLine();
void calculateError(int16_t sensorValues[]);
void calculatePID();
void commandMotors();

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
    motorL.setSpeed(100);
    motorR.setSpeed(100);
}

void loop()
{
    previousTime = currentTime;
    currentTime = micros();
    dt = (currentTime - previousTime) / 1000000.0;

    /* ----- Main part of the loop ----- */
    calculateError(lineValues); // Calculate error (difference between actual position and setpoint)

    calculatePID(); // Calculate the new movement of the robot based on the error

    commandMotors(); // Send the values calculated by the PID to the motors

    /* --------------------------------- */

    loopBlink();    // Indicate we are in main loop with short blink every 1.5 seconds
    loopRate(2000); // Set the loop speed at 2000Hz
}

void acquireLine()
{
    // 4 sensors in the middle
    lineValues[0] = qtr.getValue(2);
    lineValues[1] = qtr.getValue(3);
    lineValues[2] = qtr.getValue(4);
    lineValues[3] = qtr.getValue(5);
}

void calculateError(int16_t sensorValues[])
{
    int16_t position = 0;
    for (int i = 0; i < 4; i++)
    {
        // Weighted sum of sensor values to calculate the position
        position += (i - 1.5) * sensorValues[i];
    }
    error = position / 100; // Scaling factor for better PID performance
}

void calculatePID()
{
    // Calculate PID components
    double proportional = Kp * error;
    integral += Ki * error;
    double derivative = Kd * (error - lastError);

    // Calculate motor speeds
    motorSpeedLeft = constrain(proportional + integral + derivative, motorMinSpeed, motorMaxSpeed);
    motorSpeedRight = constrain(proportional + integral + derivative, motorMinSpeed, motorMaxSpeed);

    // Save the current error for the next iteration
    lastError = error;
}

void commandMotors()
{
    motorL.setSpeed(motorSpeedLeft);
    motorR.setSpeed(motorSpeedRight);
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
