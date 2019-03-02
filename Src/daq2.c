/*
 * daq2.c
 *
 *  Created on: Jan 26, 2019
 *      Author: Chris
 */

#include "daq2.h"
#include "DAQ_CAN.h"

extern volatile hall_sensor g_right_wheel;
extern volatile hall_sensor g_left_wheel;
extern volatile hall_sensor g_c_flow;
extern volatile DAQ_t daq;
extern uint32_t uwTick;
max1161x g_adc_mux;


/**
 * @brief initialize the peripherals of the DAQ2 system
 *
 * @param hi2c:  pointer to HAL i2c object
 * @param hi2c1: pointer to HAL i2c object
 * @param htim:  pointer to HAL timer object
 *
 * @return none
 * */
void init_daq2(volatile DAQ_t * controller, I2C_HandleTypeDef * hi2c, I2C_HandleTypeDef * hi2c1, TIM_HandleTypeDef * htim, CAN_HandleTypeDef * vcan, CAN_HandleTypeDef * dcan)
{
	controller->dcan = dcan;
	controller->vcan = vcan;
	controller->q_rx_dcan = xQueueCreate(QUEUE_SIZE_RXCAN_1, sizeof(CanRxMsgTypeDef));
	controller->q_tx_dcan = xQueueCreate(QUEUE_SIZE_TXCAN_1, sizeof(CanTxMsgTypeDef));
	controller->q_rx_vcan = xQueueCreate(QUEUE_SIZE_RXCAN_2, sizeof(CanRxMsgTypeDef));
	controller->q_tx_vcan = xQueueCreate(QUEUE_SIZE_TXCAN_2, sizeof(CanTxMsgTypeDef));

	init_hall_sensor(&g_left_wheel, htim, LEFT_WHEEL_CHANNEL);
	init_hall_sensor(&g_right_wheel, htim, RIGHT_WHEEL_CHANNEL);

	#ifdef REAR_DAQ
		init_hall_sensor(&g_c_flow, htim, C_FLOW_CHANNEL);
	#endif

	max1161x_Init(&g_adc_mux, hi2c1, MAX11614_ADDR, 7);

	DCANFilterConfig();
	VCANFilterConfig();
}
/**
 * Programmer: fallon2@purdue.edu - Chris Fallon
 *
 * @brief Creates the tasks
 *
 * @return none
 * */
void start_daq2()
{
	xTaskCreate(task_heartbeat, "HEARTBEAT", 32, NULL, 1, NULL);
	xTaskCreate(taskTX_DCAN, "TX CAN DCAN", 256, NULL, 1, NULL);
	xTaskCreate(taskTX_VCAN, "TX CAN VCAN", 256, NULL, 1, NULL);
	xTaskCreate(taskRX_VCANProcess, "RX CAN", 256, NULL, 1, NULL);
	xTaskCreate(read_adc_task, "ADC TASK", 256, &g_adc_mux, 1, NULL);
	xTaskCreate(send_wheel_speed_task, "BUFFER WHEEL SPEED", 256, NULL , 1, NULL);
	xTaskCreate(wheel_speed_zero_task, "ZERO WHEEL SPEED TASK", 256, NULL , 1, NULL);

#ifdef REAR_DAQ
	xTaskCreate(send_coolant_data_task, "COOLANT DATA TASK", 256, NULL, 1, NULL);
#endif

	HAL_GPIO_TogglePin(GPIOD, LD4_Pin);
}

/**
 * Programmer: fallon2@purdue.edu - Chris Fallon
 *
 * @brief changes the enable state of the wheel speed DAQ
 *
 * @param enable: 0 = disable, else enable
 *
 * @return none
 * */
void set_wheel_speed_capture(uint8_t enable)
{
	uint8_t prev_enable = g_left_wheel.enable;
	g_right_wheel.enable = enable;
	g_left_wheel.enable = enable;

	if (enable && !prev_enable)
	{
		vTaskResume(send_wheel_speed_task);
		vTaskResume(wheel_speed_zero_task);
	}
	else if (!enable && prev_enable)
	{
		vTaskSuspend(send_wheel_speed_task);
		vTaskSuspend(wheel_speed_zero_task);
	}
}

#ifdef REAR_DAQ
/**
 * Programmer: fallon2@purdue.edu - Chris Fallon
 *
 * @brief changes the enable state of the coolant flow DAQ
 *
 * @param enable: 0 = disable, else enable
 *
 * @return none
 * */
void set_c_flow_capture(uint8_t enable)
{
	uint8_t prev_enable = g_c_flow.enable;
	g_c_flow.enable = enable;

	if (enable && !prev_enable)
	{
		g_c_flow.speed = 0;
	}
}

/**
 * Programmer: fallon2@purdue.edu - Chris Fallon
 *
 * @brief task that adds the coolant data to CAN queue
 *
 * @return none
 * */
void send_coolant_data_task()
{
	CanTxMsgTypeDef tx;
	tx.StdId = ID_R_COOLANT;
	tx.Data[0] = 0;
	tx.Data[1] = 0;
	tx.Data[2] = 0;
	tx.Data[3] = 0;
	tx.Data[4] = 0;
	tx.Data[5] = 0;
	tx.Data[6] = (g_c_flow.speed & 0xF0) >> 8;
	tx.Data[7] = (g_c_flow.speed & 0x0F);
	tx.IDE = CAN_ID_STD;
	tx.RTR = CAN_RTR_DATA;
	tx.DLC = 8;

	xQueueSendToBack(daq.q_tx_dcan, &tx, 100);
}

#endif

/**
 * Programmer: fallon2@purdue.edu - Chris Fallon
 *
 * @brief blinks an LED
 *
 * @return none
 * */
void task_heartbeat()
{
	for (;;)
	{
		HAL_GPIO_TogglePin(GPIOD, LD5_Pin);
		vTaskDelay(HEARTBEAT_PERIOD);
	}
}


/**
 * Programmer: fallon2@purdue.edu - Chris Fallon
 *
 * @brief RTOS task to poll the ADC -> I2C mux
 *
 * @param mux: pointer to the mux object
 *
 * @return none*/
void read_adc_task(void * v_mux)
{
	//TODO add to CAN queue
	max1161x * mux = (max1161x*) v_mux;
	for(;;)
	{
//		HAL_GPIO_TogglePin(GPIOD, LD3_Pin);
		if (!mux->broke && mux->enable)
		{
			max1161x_Scan(mux, mux->data);
		}
		vTaskDelay(MUX_READ_PERIOD);
	}

}

/**
 * Programmer: fallon2@purdue.edu - Chris Fallon
 *
 * @brief RTOS task to poll wheel speed and send it to queue
 * *
 * @return none*/
void send_wheel_speed_task()
{
	CanTxMsgTypeDef tx;

	for (;;)
	{
		// if speed collection is enabled
		// both wheels should be enabled or disabled.
		// individual wheel speed toggle is not an option, so just assume that both are
		// enabled or disabled.
		if (g_left_wheel.enable)
		{
			tx.StdId = ID_WHEEL_SPEED;
			tx.Data[0] = (g_left_wheel.speed & 0xF0) >> 8;
			tx.Data[1] = (g_left_wheel.speed & 0x0F);
			tx.Data[2] = (g_right_wheel.speed & 0xF0) >> 8;
			tx.Data[3] = (g_right_wheel.speed & 0x0F);
			tx.DLC = 4;
			tx.IDE = CAN_ID_STD;
			tx.RTR = CAN_RTR_DATA;

			xQueueSendToBack(daq.q_tx_dcan, &tx, 100);
		}
		vTaskDelay(WHEEL_SPD_SEND_PERIOD);
	}

}

/**
 * Programmer: fallon2@purdue.edu - Chris Fallon
 *
 * @brief RTOS task to poll wheel speed and reset it to 0 if no pulses in a certain time frame
 * *
 * @return none*/
void wheel_speed_zero_task()
{
	for(;;)
	{
//		HAL_GPIO_TogglePin(GPIOD, LD5_Pin);
		if (g_left_wheel.enable)
		{
			if (uwTick - g_right_wheel.time_n > SPEED_ZERO_TIMEOUT)
			{
				g_right_wheel.speed = 0;
			}
			if (uwTick - g_left_wheel.time_n > SPEED_ZERO_TIMEOUT)
			{
				g_left_wheel.speed = 0;
			}
		}
		vTaskDelay(SPEED_ZERO_PERIOD);
	}

}


/**
 * Programmer: fallon2@purdue.edu - Chris Fallon
 *
 * @brief process the sensor toggling CAN message
 *
 * @param data: pointer to array which has sensor toggle bits
 *
 * @return none
 * */
void process_sensor_enable(uint8_t * data)
{
	set_wheel_speed_capture(data[0] & 0b10000000);
}
