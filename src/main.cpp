#include <Arduino.h>
#include "pinout.h"
#include "Motor/Motor.h"
#include "QTR/QTR.h"

/* Motor settings */
// Setup the motors
Motor motorL;
Motor motorR;
// Motor speed limits
const int16_t motorMinSpeed = -255;
const int16_t motorMaxSpeed = 255;

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
double lastError = 0;
double integral = 0;

/* Loop rate settings */
// Loop rate variables
float dt;
uint32_t current_time, prev_time, elapsed_time = 0;
uint32_t print_counter, serial_counter;
uint32_t blink_counter, blink_delay;
bool blinkAlternate;

/* Functions signatures definitons */
void acquireLine();
void calculatePID();
void commandMotors(uint16_t motorL, uint16_t motorR);

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
    prev_time = current_time;
    current_time = micros();
    dt = (current_time - prev_time) / 1000000.0;

    loopBlink(); // Indicate we are in main loop with short blink every 1.5 seconds

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

int16_t calculateError(int16_t sensorValues[])
{
    int16_t position = 0;
    for (int i = 0; i < 4; i++)
    {
        // Weighted sum of sensor values to calculate the position
        position += (i - 1.5) * sensorValues[i];
    }
    return position / 100; // Scaling factor for better PID performance
}

void calculatePID()
{
    // Calculate error (difference between actual position and setpoint)
    int16_t error = calculateError(lineValues);

    // Calculate PID components
    double proportional = Kp * error;
    integral += Ki * error;
    double derivative = Kd * (error - lastError);

    // Calculate motor speeds
    int16_t motorSpeedLeft = constrain(proportional + integral + derivative, motorMinSpeed, motorMaxSpeed);
    int16_t motorSpeedRight = constrain(proportional + integral + derivative, motorMinSpeed, motorMaxSpeed);

    // Update motor speeds
    commandMotors(motorSpeedLeft, motorSpeedRight);

    // Save the current error for the next iteration
    lastError = error;
}

void commandMotors(uint16_t speedL, uint16_t speedR)
{
    motorL.setSpeed(speedL);
    motorR.setSpeed(speedR);
}

void loopBlink()
{
    if (current_time - blink_counter > blink_delay)
    {
        blink_counter = micros();
        digitalWrite(LED_BUILTIN, blinkAlternate);

        if (blinkAlternate == 1)
        {
            blinkAlternate = 0;
            blink_delay = 100000;
        }
        else if (blinkAlternate == 0)
        {
            blinkAlternate = 1;
            blink_delay = 2000000;
        }
    }
}

void loopRate(int freq)
{
    float invFreq = 1.0 / freq * 1000000.0;
    uint32_t checker = micros();

    // Sit in loop until appropriate time has passed
    while (invFreq > (checker - current_time))
    {
        checker = micros();
    }
}

void printLoopRate()
{
    if (current_time - print_counter > 10000)
    {
        print_counter = micros();
        Serial.print(F("dt = "));
        Serial.println(dt * 1000000.0);
    }
}
