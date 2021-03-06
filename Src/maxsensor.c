/*
 *  maxsensor.c
 *
 *  Created on: Feb 9, 2019
 *      Author: halowens
 */
#include <math.h>
#include "maxsensor.h"

/**
 * @brief      Calculates the temperature of the Inline Flow Temp Sensor
 * @param      *tempSensor, Pointer to a sensor struct that defines the parameters of the specific sensor being read.
 */
uint8_t maxsensor_Inlineflow_Read(void * tempSensor_temp)
{
  sensor_t * tempSensor = (sensor_t *) tempSensor_temp;
  uint8_t status;
  uint16_t adcValue;

   status = max1161x_ADC_Read(tempSensor->max, tempSensor->pin, &adcValue);

   double vOut;
   double resistance;
   uint16_t knownR = FLOW_SPEED_RESISTOR_OHM; //Resistance of resistor in front of the flow Sensor

   vOut = (((double) adcValue) / 4095.0) * 4.73;
   resistance = (-knownR * vOut) / (vOut - 4.73);

   //Line of best fit calculated off of data in datasheet
   //Temperature = -26.689*ln(Resistance) + 272.279
   double r_ln = log(resistance);
   double temp = -26.689 * r_ln + 272.279;
   tempSensor->value = temp * 100;

  return status;
}

/**
 * @brief      Calculates the distance of the shock pot between 0-100mm
 * @param      *strainSensor, Pointer to a sensor struct that defines the parameters of the specific sensor being read.
 */
uint8_t maxsensor_Shockpot_Read(void * shockSensor_temp)
{
  sensor_t * strainSensor = (sensor_t *) shockSensor_temp;
  uint8_t status;
  uint16_t adcValue;
  uint8_t range = SHOCK_POT_TRAVEL_LENGTH; //maximum travel distance of shock Pot in millimeters

  status = max1161x_ADC_Read(strainSensor->max, strainSensor->pin, &adcValue);
  strainSensor->value = (adcValue/4095.0) * range;

  return status;
}
