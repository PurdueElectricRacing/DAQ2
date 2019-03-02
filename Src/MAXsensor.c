/*
 * InlineFlowTempSensor.c
 *
 *  Created on: Feb 9, 2019
 *      Author: halowens
 */
#include "max1161x.h"
#include <math.h>
#include "MAXsensor.h"

/**
 * @brief      Calculates the temperature of the Inline Flow Temp Sensor
 * @param      *inlineT, Pointer to a sensor struct that defines the parameters of the specific sensor being read.
 */
uint8_t maxsensor_Inlineflow_Read(void *sens)
{
	sensor_t * inlineT = (sensor_t *) sens;
	uint8_t status;
	uint16_t adcValue;
	float vOut;
	float resistance;
	uint16_t knownR = 10000;

	status = max1161x_ADC_Read(inlineT->max, inlineT->pin, &adcValue);

	/*
	 * This takes the read voltage value and calculates the resistance of the sensor based
	 * off the fact that there is a 10k ohm resistor in front of the sensor like in the datasheet
	 * No idea if it works but you need resistance to get the temperature value
	 */
	vOut = (adcValue / 4095.0) * 4.73;
	resistance = (-knownR * vOut) / (vOut - 4.73);
	//r = 10k/((5/v)-1)
	//Temperature = -26.689*ln(x) + 272.279
	inlineT->value = -26.689 * log(resistance) + 272.279;
	return status;

}

