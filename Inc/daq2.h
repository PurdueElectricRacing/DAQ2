/*
 * daq2.h
 *
 *  Created on: Jan 26, 2019
 *      Author: Chris
 */

#ifndef DAQ2_H_
#define DAQ2_H_

#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "hall_effect_sensors.h"
#include "max1161x.h"
#include "DAQ_CAN.h"

#define REAR_DAQ		// undefine this if this is the front DAQ board

#ifdef REAR_DAQ
#define ID_WHEEL_SPEED ID_R_WHEEL_SPEED
#endif

#define MUX_READ_PERIOD       10  / portTICK_RATE_MS 	// 100hz
#define HEARTBEAT_PERIOD      250 / portTICK_RATE_MS	// 2.5hz
#define WHEEL_SPD_SEND_PERIOD 10  / portTICK_RATE_MS  // 100hz
#define SPEED_ZERO_PERIOD		  250 / portTICK_RATE_MS	// 2.5hz
#define SPEED_ZERO_TIMEOUT    100 	// 100ms

typedef struct DAQ_t
{
	CAN_HandleTypeDef * vcan;
	CAN_HandleTypeDef * dcan;

	QueueHandle_t			q_rx_dcan;
	QueueHandle_t			q_tx_dcan;
	QueueHandle_t			q_rx_vcan;
	QueueHandle_t			q_tx_vcan;
}DAQ_t;

void init_daq2(volatile DAQ_t * controller, I2C_HandleTypeDef * hi2c, I2C_HandleTypeDef * hi2c1, TIM_HandleTypeDef * htim, CAN_HandleTypeDef * vcan, CAN_HandleTypeDef * dcan);
void read_adc_task(void * v_mux);
void wheel_speed_zero_task();
void send_wheel_speed_task();
void process_sensor_enable(uint8_t * data);
void task_heartbeat();
void start_daq2();
void set_wheel_speed_capture(uint8_t enable);
void send_coolant_data_task();




#endif /* DAQ2_H_ */
