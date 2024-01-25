// Figures.h

#include <Arduino.h>

/**
 * @brief Function to make the robot turn left after a mark.
 * WARNING: Only works in this case ┱
 *
 */
void turnLeft()
{
    while (qtr.isBlack(2) || qtr.isBlack(3) || qtr.isBlack(4) || qtr.isBlack(5))
    {
        motorL.stop();
        motorR.setSpeed(100);
    }
    while (qtr.isWhite(2) || qtr.isWhite(3) || qtr.isWhite(4) || qtr.isWhite(5))
    {
        motorL.stop();
        motorR.setSpeed(100);
    }
    if (qtr.isBlack(3) || qtr.isBlack(4))
    {
        motorL.setSpeed(100);
        motorR.setSpeed(100);
    }
}

/**
 * @brief Function to make the robot turn right after a mark.
 * WARNING: Only works in this case ┲
 *
 */
void turnRight()
{
    while (qtr.isBlack(2) || qtr.isBlack(3) || qtr.isBlack(4) || qtr.isBlack(5))
    {
        motorL.setSpeed(100);
        motorR.stop();
    }
    while (qtr.isWhite(2) || qtr.isWhite(3) || qtr.isWhite(4) || qtr.isWhite(5))
    {
        motorL.setSpeed(100);
        motorR.stop();
    }
    if (qtr.isBlack(3) || qtr.isBlack(4))
    {
        motorL.setSpeed(100);
        motorR.setSpeed(100);
    }
}

/**
 * @brief Function to make the robot do a 360° after a mark.
 * WARNING: Only works in the middle of a straight line
 *
 */
void turn360()
{
    while (qtr.isBlack(2) || qtr.isBlack(3) || qtr.isBlack(4) || qtr.isBlack(5))
    {
        motorL.setSpeed(100);
        motorR.stop();
    }
    while (qtr.isWhite(2) || qtr.isWhite(3) || qtr.isWhite(4) || qtr.isWhite(5))
    {
        motorL.setSpeed(100);
        motorR.stop();
    }
    while (qtr.isBlack(2) || qtr.isBlack(3) || qtr.isBlack(4) || qtr.isBlack(5))
    {
        motorL.setSpeed(100);
        motorR.stop();
    }
    while (qtr.isWhite(2) || qtr.isWhite(3) || qtr.isWhite(4) || qtr.isWhite(5))
    {
        motorL.setSpeed(100);
        motorR.stop();
    }

    if (qtr.isBlack(3) || qtr.isBlack(4))
    {
        motorL.setSpeed(100);
        motorR.setSpeed(100);
    }
}
