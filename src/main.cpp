#include <Arduino.h>
#include <pinout.h>

/* Motors variables */
#define MOTOR_L_FWD 1
#define MOTOR_L_STOP 0
#define MOTOR_L_BWD -1

#define MOTOR_R_FWD 1
#define MOTOR_R_STOP 0
#define MOTOR_R_BWD -1

int8_t motorLRotation;
int8_t motorRRotation;

void setMotorsRotation(uint8_t l, uint8_t r);
void setMotorsSpeed(uint8_t l, uint8_t r);

void setup()
{
    Serial.begin(115200); // Init the Serial communication

    setInOut(); // Define every pinMode()

    setMotorsRotation(MOTOR_L_STOP, MOTOR_R_STOP); // Init the motor rotation at STOP
    setMotorsSpeed(0, 0);                          // Init the motor speed at 0
}

void loop()
{
}

void setMotorsRotation(uint8_t l, uint8_t r)
{
    // Motor L rotation
    if (l == 1)
    {
        digitalWrite(fwdL, HIGH);
        digitalWrite(bwdL, LOW);
    }
    else if (l == 0)
    {
        digitalWrite(fwdL, LOW);
        digitalWrite(bwdL, LOW);
    }
    else if (l == -1)
    {
        digitalWrite(fwdL, LOW);
        digitalWrite(bwdL, HIGH);
    }

    // Motor R rotation
    if (r == 1)
    {
        digitalWrite(fwdR, HIGH);
        digitalWrite(bwdR, LOW);
    }
    else if (r == 0)
    {
        digitalWrite(fwdR, LOW);
        digitalWrite(bwdR, LOW);
    }
    else if (r == -1)
    {
        digitalWrite(fwdR, LOW);
        digitalWrite(bwdR, HIGH);
    }
}

void setMotorsSpeed(uint8_t l, uint8_t r)
{
    analogWrite(enL, l);
    analogWrite(enR, r);
}
