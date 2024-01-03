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
}
