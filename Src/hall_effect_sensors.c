/***************************************************************************
**     Name of File: hall_effect_sensors.c
*
*     Authors (Include Email):
*       1. Name  Chris Fallon     Email fallon2@purdue.edu
*
*     File dependents: (header files, flow charts, referenced documentation)
*       1. hall_effect_sensors.h, main.h, stm32f4xx_hal.h
*
*     File Description: handles the timer interrupts
*
*
***************************************************************************/
#include "hall_effect_sensors.h"

extern volatile hall_sensor g_right_wheel;
extern volatile hall_sensor g_left_wheel;
#ifdef REAR_DAQ
  extern volatile hall_sensor g_c_flow;
#endif


/**
 *	@brief: Initializes the provided hall_sensor struct with the provided TIM_HandleTypeDef pointer
 *				and initializes all values to 0. Starts the Input Capture for the provided TIM_HandleTypeDef *
 *				in interrupt mode on channel 2 of the timer.
 *
 *		@param sensor:  pointer to the hall_sensor object
 *		@param htim:    pointer to HAL timer object
 *		@param channel: sensor's timer channel
 *
 *		@retvalue: none
 *	  **/
void init_hall_sensor(volatile hall_sensor *sensor, TIM_HandleTypeDef *htim, uint32_t channel)
{
	sensor->htim = htim;
	sensor->time_n = 0;
	sensor->time_n_minus_1 = 0;
	sensor->speed = 0;
	sensor->error = 0;

	if (HAL_TIM_IC_Start_IT(htim, channel) != HAL_OK)
	{
		sensor->error = 1;
	}
}

/**
 * 	-- Input Compare capture callback handler
 *
 *		@param: TIM_HandleTypeDef *htim
 *		@brief: determines which timer is being interrupted on
 *		@retvalue: none
 *
 **/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
	{
		calculate_and_store_speed(&g_left_wheel, LEFT_WHEEL_CHANNEL);
	}
	else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
		calculate_and_store_speed(&g_right_wheel, RIGHT_WHEEL_CHANNEL);
	}
#ifdef REAR_DAQ
	else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		calculate_and_store_speed(&g_c_flow, C_FLOW_CHANNEL);
	}
#endif
}

/**
 * @brief: calculates and stores the speed of the sensor pointed to by *sensor
 *
 * @param: hall_sensor *sensor: pointer to the hall effect sensor
 *
 * @return: none
 * */

void calculate_and_store_speed(volatile hall_sensor *sensor, uint32_t channel)
{
	if (sensor->error)
	{
		return;
	}

	uint32_t dt = 0;

	//get the current timer value for the sensor
	sensor->time_n_minus_1 = sensor->time_n;
	sensor->time_n = __HAL_TIM_GetCompare(sensor->htim, channel);		//current time gets stored in the sensor's respective channel
	dt = sensor->time_n - sensor->time_n_minus_1;
	if (dt > 0)
	{
		if (channel == RIGHT_WHEEL_CHANNEL || channel == LEFT_WHEEL_CHANNEL)
		{
			sensor->speed = calculate_wheel_speed(dt);
		}
		else if (channel == C_FLOW_CHANNEL)
		{
			sensor->speed = calculate_flow_rate(dt);
		}
	}

}

/**
 * @brief: calculates the wheel speed multiplied by 10000
 *
 * @param: uint32_t dt: time between measurements
 *
 * @return: calculated wheel speed
 * */
uint32_t calculate_wheel_speed(uint32_t dt)
{
	//if the time delta > 0, then we calculate speed, otherwise we divide by 0
	 float speed = ((MILLIS_TO_MINUTES * ( TOOTH_ANGLE / dt) ) / FULL_CIRCLE);
	 return (uint32_t) (speed);
}

/**
 * @brief: calculates the flow rate the final multiplied by 10000
 *
 * @param: uint32_t dt: time between measurements
 *
 * @return: calculated flow rate
 * */
uint32_t calculate_flow_rate(uint32_t dt)
{
	// ((pulses per millis) * 1000) = HZ
	// HZ / 7.5 = L/MIN
	float speed = (((1.0f / dt) * 1000.0f) / LPM_CONVERSION );
	return (uint32_t) (speed);
}
