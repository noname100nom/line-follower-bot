// QTR.h

#include <Arduino.h>

#ifndef QTR_h
#define QTR_h

class QTR
{
public:
    /**
     * @brief Default constructor for the QTR class.
     */
    QTR();

    /**
     * @brief Function to set the pins for the QTR sensor
     *
     * @param en Pin number for the QTR enable
     * @param d1 Pin number for the first sensor
     * @param d2 Pin number for the second sensor
     * @param d3 Pin number for the third sensor
     * @param d4 Pin number for the fourth sensor
     * @param d5 Pin number for the fifth sensor
     * @param d6 Pin number for the sixth sensor
     * @param d7 Pin number for the seventh sensor
     * @param d8 Pin number for the eighth sensor
     */
    void init(uint8_t en, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7, uint8_t d8);

    /**
     * @brief Function to get the value of the selected sensor
     *
     * @param sensor Selection of the sensor (0-7 ; 0 indexed)
     * @return uint16_t Value of the sensor
     */
    uint16_t getValue(uint8_t sensor);

    /**
     * @brief Function to calibrate the QTR sensor
     *
     * @param time Time in ms for the calibration
     */
    void calibrate(uint16_t time);

    /**
     * @brief Function to know if the color under the sensor is black
     * 
     * @param sensor Selection of the sensor (0-7 ; 0 indexed)
     * @return true True if the color is black
     * @return false False if the color is white
     */
    bool isBlack(uint8_t sensor);

    /**
     * @brief Function to know if the color under the sensor is white
     * 
     * @param sensor Selection of the sensor (0-7 ; 0 indexed)
     * @return true True if the color is white
     * @return false False if the color is black
     */
    bool isWhite(uint8_t sensor);

    /**
     * @brief Function to print the value for the white, black
     * and the treshold value (needs Serial.begin()).
     * 
     */
    void printInfos();

    /**
     * @brief Function to print the array of sensors. Prints
     * 1 if black or 0 if white (needs Serial.begin()).
     * 
     */
    void printArray();

    /**
     * @brief Function to print the analog value of the array of
     * sensors (needs Serial.begin()).
     * 
     */
    void printAnalogArray();

private:
    uint8_t en;
    uint8_t sensorPin[8];

    uint16_t whiteVal;
    uint16_t blackVal;
    uint16_t treshold;

    uint32_t qtrCurrentTime;
    uint32_t qtrPrintCounter;
};

#endif
