// Motor.cpp

#include <Arduino.h>
#include "QTR.h"

QTR::QTR()
{
    // Default constructor
}

void QTR::init(uint8_t en, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7, uint8_t d8)
{
    // Set the pins for the qtr sensor
    this->en = en;
    this->sensorPin[0] = d8;
    this->sensorPin[1] = d7;
    this->sensorPin[2] = d6;
    this->sensorPin[3] = d5;
    this->sensorPin[4] = d4;
    this->sensorPin[5] = d3;
    this->sensorPin[6] = d2;
    this->sensorPin[7] = d1;

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

    digitalWrite(en, HIGH);
}

uint16_t QTR::getValue(uint8_t sensor)
{
    if (sensor >= 0 && sensor < 8)
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
    qtrCurrentTime = micros();
    if (qtrCurrentTime - qtrPrintCounter > 10000)
    {
        qtrPrintCounter = micros();

        Serial.printf("White: %d | Black: %d | Treshold: %d\n", whiteVal, blackVal, treshold);
    }
}

void QTR::printArray()
{
    qtrCurrentTime = micros();
    if (qtrCurrentTime - qtrPrintCounter > 10000)
    {
        qtrPrintCounter = micros();

        Serial.printf("| %d | %d | %d | %d | %d | %d | %d | %d |\n", isBlack(0), isBlack(1), isBlack(2), isBlack(3), isBlack(4), isBlack(5), isBlack(6), isBlack(7));
    }
}

void QTR::printAnalogArray()
{
    qtrCurrentTime = micros();
    if (qtrCurrentTime - qtrPrintCounter > 10000)
    {
        qtrPrintCounter = micros();

        Serial.printf("| %04d | %04d | %04d | %04d | %04d | %04d | %04d | %04d |\n", getValue(0), getValue(1), getValue(2), getValue(3), getValue(4), getValue(5), getValue(6), getValue(7));
    }
}
