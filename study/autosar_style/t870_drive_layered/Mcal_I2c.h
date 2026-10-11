// ============================================================
// MCAL I2c: A4/A5 마스터
// ============================================================

#ifndef MCAL_I2C_H
#define MCAL_I2C_H

#include "Std_Types.h"

void    Mcal_I2c_Init();
uint8_t Mcal_I2c_RequestFrom(uint8_t address, uint8_t length);
int     Mcal_I2c_Available();
int     Mcal_I2c_Read();

#endif
