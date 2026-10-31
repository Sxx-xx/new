#include <Arduino.h>
#include <Wire.h>
#include "Mcal_I2c.h"


void Mcal_I2c_Init()
{
  Wire.begin();
}


uint8_t Mcal_I2c_RequestFrom(uint8_t address, uint8_t length)
{
  return Wire.requestFrom((int)address, (int)length);
}


int Mcal_I2c_Available()
{
  return Wire.available();
}


int Mcal_I2c_Read()
{
  return Wire.read();
}
