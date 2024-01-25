#include <Arduino.h>
#include "NewLiquidCrystal-master/LiquidCrystal_I2C.h"

LiquidCrystal_I2C lcd(0x38);  // Set the LCD I2C address

void setup()
{
    lcd.begin(16,2);
}

void loop()
{
}
