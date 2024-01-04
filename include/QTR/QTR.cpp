// Motor.cpp

#include "QTR.h"
#include "Arduino.h"

QTR::QTR()
{
    // Default constructor
}

void QTR::init(uint8_t en, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7, uint8_t d8)
{
    // Set the pins for the qtr sensor
    this->en = en;
    this->sensorPin[0] = d1;
    this->sensorPin[1] = d2;
    this->sensorPin[2] = d3;
    this->sensorPin[3] = d4;
    this->sensorPin[4] = d5;
    this->sensorPin[5] = d6;
    this->sensorPin[6] = d7;
    this->sensorPin[7] = d8;

    // Configure pins as INPUT and OUTPUTS
    pinMode(en, OUTPUT);
    pinMode(sensorPin[0], INPUT);
    pinMode(sensorPin[1], INPUT);
    pinMode(sensorPin[2], INPUT);
    pinMode(sensorPin[3], INPUT);
    pinMode(sensorPin[4], INPUT);
    pinMode(sensorPin[5], INPUT);
    pinMode(sensorPin[6], INPUT);
    pinMode(sensorPin[7], INPUT);
}

uint16_t QTR::getValue(uint8_t sensor)
{
    if (sensor < 8)
    {
        return analogRead(sensorPin[sensor]);
    }

    return 0; // Handle invalid sensor number (out of range)
}

void QTR::calibrate(uint16_t time)
{
    whiteVal = analogRead(sensorPin[0]);
    blackVal = analogRead(sensorPin[0]);

    uint16_t currVal;

    uint32_t c = 0;
    while (c < time)
    {
        for (uint8_t i = 0; i < 8; i++)
        {
            currVal = analogRead(sensorPin[i]);

            if (currVal < whiteVal)
            {
                whiteVal = currVal;
            }
            else if (currVal > blackVal)
            {
                blackVal = currVal;
            }
        }

        treshold = (whiteVal + blackVal) / 2;

        delay(1);
        c++;
    }
}

bool QTR::isBlack(uint8_t sensor)
{
    if (getValue(sensor) > treshold)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool QTR::isWhite(uint8_t sensor)
{
    if (getValue(sensor) < treshold)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void QTR::printInfos()
{
    if (qtrCurrentTime - qtrPrintCounter > 10000)
    {
        qtrPrintCounter = micros();
        
        Serial.printf("White: %d | Black: %d | Treshold: %d\n", whiteVal, blackVal, treshold);
    }
}

void QTR::printArray()
{
    if (qtrCurrentTime - qtrPrintCounter > 10000)
    {
        qtrPrintCounter = micros();

        Serial.printf("| %04d | %04d | %04d | %04d | %04d | %04d | %04d | %04d |\n", sensorPin[0], sensorPin[1], sensorPin[2], sensorPin[3], sensorPin[4], sensorPin[5], sensorPin[6], sensorPin[7]);
    }
}
