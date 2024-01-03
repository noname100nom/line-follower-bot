#include <Arduino.h>
#include "pinout.h"
#include "Motor/Motor.h"
#include "QTR/QTR.h"

Motor motorL;
Motor motorR;

QTR qtr;

/* Motors variables */
int8_t motorLSpeed; // -255 to 255 ; 0 is STOP
int8_t motorRSpeed; // -255 to 255 ; 0 is STOP

float dt;
uint32_t current_time, prev_time, elapsed_time = 0;
uint32_t print_counter, serial_counter;
uint32_t blink_counter, blink_delay;
bool blinkAlternate;

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
}

void loop()
{
    prev_time = current_time;
    current_time = micros();
    dt = (current_time - prev_time) / 1000000.0;

    loopBlink(); // Indicate we are in main loop with short blink every 1.5 seconds

    loopRate(2000); // Set the loop speed at 2000Hz
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
