// Motor.h

#include <Arduino.h>

#ifndef Motor_h
#define Motor_h

class Motor
{
public:
    /**
     * @brief Default constructor for the Motor class.
     */
    Motor();

    /**
     * @brief Function to set pins for the motor.
     *
     * @param en Pin number for the motor enable.
     * @param fwd Pin number for the first input.
     * @param bwd Pin number for the second input.
     */
    void init(uint8_t en, uint8_t fwd, uint8_t bwd);

    /**
     * @brief Function to set the motor speed.
     *
     * @param speed Motor speed (-255 to 255).
     */
    void setSpeed(int16_t speed);

    /**
     * @brief Function to stop the motor.
     */
    void stop();

private:
    uint8_t en;  ///< Motor enable pin.
    uint8_t fwd; ///< Motor forward pin.
    uint8_t bwd; ///< Motor backward pin.
};

#endif
