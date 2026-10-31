#include <Arduino.h>
#include "Mcal_Adc.h"
#include "Mcal_Cfg.h"


int Mcal_Adc_ReadSteerPot()
{
  return analogRead(MCAL_PIN_STEER_POT);
}
