#include <Arduino.h>
#include "pinout.h"
#include "Motor/Motor.h"

Motor motorL;
Motor motorR;

/* Motors variables */
int8_t motorLSpeed; // -255 to 255 ; 0 is STOP
int8_t motorRSpeed; // -255 to 255 ; 0 is STOP

void setup()
{
    Serial.begin(115200); // Init the Serial communication

    setInOut(); // Define every pinMode()

    // Init the motors speed at 0
    motorL.stop();
    motorR.stop();
}

void loop()
{
}
