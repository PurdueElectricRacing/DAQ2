/*
 * InlineFlowTempSensor.h
 *
 *  Created on: Feb 9, 2019
 *      Author: halowens
 */

#ifndef MAXSENSOR_H_
#define MAXSENSOR_H_

#include "max1161x.h"
#include "daq2.h"


typedef struct {
	uint8_t pin; 					// Pin on the MAX Chip
	max1161x* max; 				// When were using both chips this can define which one is being used
	uint16_t value;				// holds the value of sensor is updated by calling the function below
	uint8_t (*read)(void * p_self); //Should be called to get sensor value
}sensor_t;

uint8_t maxsensor_Inlineflow_Read(void *);

#endif /* MAXSENSOR_H_ */
