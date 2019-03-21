/*
 * daq2.c
 *
 *  Created on: Jan 26, 2019
 *      Author: Chris
 */

#include <maxsensor.h>
#include "daq2.h"
#include "DAQ_CAN.h"
#include "max1161x.h"

extern volatile hall_sensor g_right_wheel;
extern volatile hall_sensor g_left_wheel;
extern volatile hall_sensor g_c_flow;
extern volatile DAQ_t daq;
extern uint32_t uwTick;
max1161x g_max11614;
max1161x g_max11616;
sensor_t g_max11614_sensors[MAX11614_CHANNELS];
sensor_t g_max11616_sensors[MAX11616_CHANNELS];


/**
 * @brief initialize the peripherals of the DAQ2 system
 *
 * @param hi2c:  pointer to HAL i2c object
 * @param hi2c1: pointer to HAL i2c object
 * @param htim:  pointer to HAL timer object
 *
 * @return none
 * */
void init_daq2(volatile DAQ_t * controller, I2C_HandleTypeDef * hi2c, I2C_HandleTypeDef * hi2c1,
						TIM_HandleTypeDef * htim, CAN_HandleTypeDef * vcan, CAN_HandleTypeDef * dcan)
{
	daq.dcan = dcan;
	daq.vcan = vcan;
	daq.q_rx_dcan = xQueueCreate(QUEUE_SIZE_RXCAN_1, sizeof(CanRxMsgTypeDef));
	daq.q_tx_dcan = xQueueCreate(QUEUE_SIZE_TXCAN_1, sizeof(CanTxMsgTypeDef));
	daq.q_rx_vcan = xQueueCreate(QUEUE_SIZE_RXCAN_2, sizeof(CanRxMsgTypeDef));
	daq.q_tx_vcan = xQueueCreate(QUEUE_SIZE_TXCAN_2, sizeof(CanTxMsgTypeDef));
  daq.can_enable = 0b11111111;

	init_hall_sensor(&g_left_wheel, htim, LEFT_WHEEL_CHANNEL);
	init_hall_sensor(&g_right_wheel, htim, RIGHT_WHEEL_CHANNEL);

	// init the max chips
	max1161x_Init(&g_max11614, hi2c, MAX11614_ADDR, 7);
	max1161x_Init(&g_max11616, hi2c, MAX11616_ADDR, 7);

	init_max_arrays();

	#ifdef REAR_DAQ
		init_hall_sensor(&g_c_flow, htim, C_FLOW_CHANNEL);
	#endif

	DCANFilterConfig();
	VCANFilterConfig();
}

/**
 * Programmer: fallon2@purdue.edu - Chris Fallon
 *
 * @brief Initializes the array of sensors which are connected to
 * 				the MAX1161x chip.
 *
 * @param sensor_array: pointer to the array of sensors
 * @param size: length of the corresponding array
 *
 * @return None.
 * */
void init_max_arrays()
{
	// assign each sensor its maxim pointer
	uint8_t i;
	for (i = 0; i < MAX11614_CHANNELS; i++)
	{
		g_max11614_sensors[i].max = &g_max11614;
	}
	for (i = 0; MAX11616_CHANNELS; i++)
	{
		g_max11616_sensors[i].max = &g_max11616;
	}

	g_max11614_sensors[STEER_TIE_ROD_RIGHT].pin = STEER_TIE_ROD_RIGHT;
	g_max11614_sensors[STEER_TIE_ROD_RIGHT].read = &maxsensor_Straingauge_Read;
	g_max11614_sensors[STEER_TIE_ROD_LEFT].pin = STEER_TIE_ROD_LEFT;
	g_max11614_sensors[STEER_TIE_ROD_LEFT].read = &maxsensor_Straingauge_Read;
	g_max11614_sensors[PUSH_ROD_RIGHT].pin = PUSH_ROD_RIGHT;
	g_max11614_sensors[PUSH_ROD_RIGHT].read = &maxsensor_Straingauge_Read;
	g_max11614_sensors[PUSH_ROD_LEFT].pin = PUSH_ROD_LEFT;
	g_max11614_sensors[PUSH_ROD_LEFT].read = &maxsensor_Straingauge_Read;
	g_max11614_sensors[LCA_L_BACK].pin = LCA_L_BACK;
	g_max11614_sensors[LCA_L_BACK].read = &maxsensor_Straingauge_Read;
	g_max11614_sensors[LCA_L_FRONT].pin = LCA_L_FRONT;
	g_max11614_sensors[LCA_L_FRONT].read = &maxsensor_Straingauge_Read;
	g_max11614_sensors[UCA_L_BACK].pin = UCA_L_BACK;
	g_max11614_sensors[UCA_L_BACK].read = &maxsensor_Straingauge_Read;
	g_max11614_sensors[UCA_L_FRONT].pin = UCA_L_FRONT;
	g_max11614_sensors[UCA_L_FRONT].read = &maxsensor_Straingauge_Read;

	g_max11616_sensors[UCA_R_BACK].pin = UCA_R_BACK;
	g_max11616_sensors[UCA_R_BACK].read = &maxsensor_Straingauge_Read;
	g_max11616_sensors[UCA_R_FRONT].pin = UCA_R_FRONT;
	g_max11616_sensors[UCA_R_FRONT].read = &maxsensor_Straingauge_Read;
	g_max11616_sensors[LCA_R_FRONT].pin = LCA_R_FRONT;
	g_max11616_sensors[LCA_R_FRONT].read = &maxsensor_Straingauge_Read;
	g_max11616_sensors[LCA_R_BACK].pin = LCA_R_BACK;
	g_max11616_sensors[LCA_R_BACK].read = &maxsensor_Straingauge_Read;
	g_max11616_sensors[MOTOR_C_TEMP].pin = MOTOR_C_TEMP;
	g_max11616_sensors[MOTOR_C_TEMP].read = &maxsensor_Inlineflow_Read;
	g_max11616_sensors[RAD_C_TEMP].pin = RAD_C_TEMP;
	g_max11616_sensors[RAD_C_TEMP].read = &maxsensor_Inlineflow_Read;
	g_max11616_sensors[MOTOR_CONT_C_TEMP].pin = MOTOR_CONT_C_TEMP;
	g_max11616_sensors[MOTOR_CONT_C_TEMP].read = &maxsensor_Inlineflow_Read;
	g_max11616_sensors[LEFT_SHOCK_POT].pin = LEFT_SHOCK_POT;

	g_max11616_sensors[LEFT_SHOCK_POT].read = NULL;

	g_max11616_sensors[ARB_DROPLINK_LEFT].pin = ARB_DROPLINK_LEFT;
	g_max11616_sensors[ARB_DROPLINK_LEFT].read = &maxsensor_Straingauge_Read;
	g_max11616_sensors[ARB_DROPLINK_RIGHT].pin = ARB_DROPLINK_RIGHT;
	g_max11616_sensors[ARB_DROPLINK_RIGHT].read = &maxsensor_Straingauge_Read;
	g_max11616_sensors[ARB_TORSIONAL].pin = ARB_TORSIONAL;
	g_max11616_sensors[ARB_TORSIONAL].read = &maxsensor_Straingauge_Read;
	g_max11616_sensors[RIGHT_SHOCK_POT].pin = RIGHT_SHOCK_POT;

	g_max11616_sensors[RIGHT_SHOCK_POT].read = NULL;
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
	// @TODO optimize stack size for each function
	xTaskCreate(task_heartbeat, "HEARTBEAT", 4, NULL, 1, NULL);
	xTaskCreate(taskTX_DCAN, "TX CAN DCAN", 256, NULL, 1, NULL);
	xTaskCreate(taskTX_VCAN, "TX CAN VCAN", 256, NULL, 1, NULL);
	xTaskCreate(taskRX_VCANProcess, "RX CAN", 256, NULL, 1, NULL);

	if (!g_max11614.broke && !g_max11616.broke)
	{
//		xTaskCreate(read_adc_task, "ADC TASK", 256, NULL, 1, NULL);
		xTaskCreate(send_uca_data, "UCA DATA TASK", 256, NULL, 1, NULL);
		xTaskCreate(send_lca_data, "LCA_DATA_TASK", 256, NULL, 1, NULL);
		xTaskCreate(send_shock_data, "SHOCK DATA TASK", 256, NULL, 1, NULL);
		xTaskCreate(send_arb_torsional_data, "ARB TORSIONAL DATA", 256, NULL, 1, NULL);
		xTaskCreate(send_push_rod_data, "PUSH ROD DATA TASK", 256, NULL, 1, NULL);
	}
	if (!g_left_wheel.error && !g_right_wheel.error)
	{
		xTaskCreate(send_wheel_speed_task, "BUFFER WHEEL SPEED", 256, NULL, 1, NULL);
		xTaskCreate(wheel_speed_zero_task, "ZERO WHEEL SPEED TASK", 256, NULL, 1, NULL);
	}

#ifdef REAR_DAQ
	if (!g_c_flow.error)
	{
		xTaskCreate(send_coolant_data_task, "COOLANT DATA TASK", 256, NULL, 1, NULL);
	}
	else
	{
		xTaskCreate(error_task, "ERROR TASK", 256, NULL, 1, NULL);
	}
#else
	xTaskCreate(send_drop_link_data, "DROP LINK DATA TASK", 256, NULL, 1, NULL);
#endif

	if (g_left_wheel.error || g_right_wheel.error || g_max11614.broke || g_max11616.broke)
	{
		xTaskCreate(error_task, "ERROR TASK", 256, NULL, 1, NULL);
	}

	HAL_GPIO_TogglePin(GPIOD, LD4_Pin);
}


/**
 * Programmer: fallon2@purdue.edu - Chris Fallon
 *
 * @brief send error message over CAN if there is an error on any of the sensors
 *
 * @return none
 * */
void error_task(){

	while (PER == GREAT)
	{
		CanTxMsgTypeDef tx;
		tx.StdId = ERROR_ID;
		tx.IDE = CAN_ID_STD;
		tx.RTR = CAN_RTR_DATA;
		tx.DLC = 5;
		tx.Data[L_WHEEL_SPD_ERROR] = g_left_wheel.error;
		tx.Data[R_WHEEL_SPD_ERROR] = g_right_wheel.error;
		tx.Data[MAX11614_ERROR] = g_max11614.broke;
		tx.Data[MAX11616_ERROR] = g_max11616.broke;

		#ifdef REAR_DAQ
			tx.Data[COOL_SPD_ERROR] = g_c_flow.error;
		#endif

		xQueueSendToBack(daq.q_tx_dcan, &tx, 100);
		vTaskDelay(ERROR_MSG_PERIOD);
	}
}

/**
 * Programmer: fallon2@purdue.edu - Chris Fallon
 *
 * @brief changes the enable state provided sensor grouping
 *
 * @param enable: 0 = disable, else enable
 *
 * @return none
 * */
void set_sensor_capture(uint8_t enable, TaskFunction_t function, uint8_t mask)
{
	// get the previous state of the enable bit
	uint8_t prev_enable = daq.can_enable & mask;
	// new status of enable bit
	// example: can_enable == 0b11111111
	// 					enable     == 0b00000000
	//          new_enable =  0b11111111 ^ 0b00000000 == 0b11111111
	daq.can_enable = enable ^ daq.can_enable;

	if (enable && !prev_enable)
	{
		if (function == send_wheel_speed_task)
		{
			vTaskResume(wheel_speed_zero_task);
		}
		vTaskResume(function);
	}
	else if (!enable && prev_enable)
	{
		if (function == send_wheel_speed_task)
		{
			vTaskResume(wheel_speed_zero_task);
		}
		vTaskSuspend(function);
	}
}

/**
 * Programmer: fallon2@purdue.edu - Chris Fallon
 *
 * @brief task to send shock pot data
 *
 * @return none
 * */
void send_shock_data()
{
	while (PER == GREAT)
	{
		CanTxMsgTypeDef tx;
		tx.StdId = SHOCK_POT_ID;
		tx.Data[0] = (g_max11616_sensors[LEFT_SHOCK_POT].value >> 8) & 0x0F;
		tx.Data[1] = (g_max11616_sensors[LEFT_SHOCK_POT].value) & 0x0F;
		tx.Data[2] = (g_max11616_sensors[RIGHT_SHOCK_POT].value >> 8) & 0x0F ;
		tx.Data[3] = (g_max11616_sensors[RIGHT_SHOCK_POT].value) & 0x0F;
		tx.IDE = CAN_ID_STD;
		tx.RTR = CAN_RTR_DATA;
		tx.DLC = 4;
		xQueueSendToBack(daq.q_tx_dcan, &tx, 100);
		vTaskDelay(REFRESH_RATE);
	}
}

/**
 * Programmer: fallon2@purdue.edu - Chris Fallon
 *
 * @brief task to send arb torsional data
 *
 * @return none
 * */
void send_arb_torsional_data()
{
	while (PER == GREAT)
	{
		CanTxMsgTypeDef tx;
		tx.StdId = ARB_TORSIONAL_ID;
		tx.Data[0] = (g_max11614_sensors[ARB_TORSIONAL].value >> 8) & 0x0F;
		tx.Data[1] = (g_max11614_sensors[ARB_TORSIONAL].value) & 0x0F;
		tx.IDE = CAN_ID_STD;
		tx.RTR = CAN_RTR_DATA;
		tx.DLC = 2;
		xQueueSendToBack(daq.q_tx_dcan, &tx, 100);
		vTaskDelay(REFRESH_RATE);
	}
}

/**
 * Programmer: fallon2@purdue.edu - Chris Fallon
 *
 * @brief task to send upper control arm data
 *
 * @return none
 * */
void send_uca_data()
{
	while (PER == GREAT)
	{
		CanTxMsgTypeDef tx;
		tx.StdId = UCA_ID;
		tx.Data[0] = (g_max11614_sensors[UCA_L_FRONT].value >> 8) & 0x0F;
		tx.Data[1] = (g_max11614_sensors[UCA_L_FRONT].value) & 0x0F;
		tx.Data[2] = (g_max11614_sensors[UCA_L_BACK].value >> 8) & 0x0F;
		tx.Data[3] = (g_max11614_sensors[UCA_L_BACK].value) & 0x0F;
		tx.Data[4] = (g_max11616_sensors[UCA_R_FRONT].value >> 8) & 0x0F;
		tx.Data[5] = (g_max11616_sensors[UCA_R_FRONT].value) & 0x0F;
		tx.Data[6] = (g_max11616_sensors[UCA_R_BACK].value >> 8) & 0x0F;
		tx.Data[7] = (g_max11616_sensors[UCA_R_BACK].value) & 0x0F;
		tx.IDE = CAN_ID_STD;
		tx.RTR = CAN_RTR_DATA;
		tx.DLC = 8;

		xQueueSendToBack(daq.q_tx_dcan, &tx, 100);
		vTaskDelay(REFRESH_RATE);
	}
}

/**
 * Programmer: fallon2@purdue.edu - Chris Fallon
 *
 * @brief task to send lower control arm data
 *
 * @return none
 * */
void send_lca_data()
{
	while (PER == GREAT)
	{
		CanTxMsgTypeDef tx;
		tx.StdId = LCA_ID;
		tx.Data[0] = (g_max11614_sensors[LCA_L_FRONT].value >> 8) & 0x0F;
		tx.Data[1] = (g_max11614_sensors[LCA_L_FRONT].value) & 0x0F;
		tx.Data[2] = (g_max11614_sensors[LCA_L_BACK].value >> 8) & 0x0F;
		tx.Data[3] = (g_max11614_sensors[LCA_L_BACK].value) & 0x0F;
		tx.Data[4] = (g_max11616_sensors[LCA_R_FRONT].value >> 8) & 0x0F;
		tx.Data[5] = (g_max11616_sensors[LCA_R_FRONT].value) & 0x0F;
		tx.Data[6] = (g_max11616_sensors[LCA_R_BACK].value >> 8) & 0x0F;
		tx.Data[7] = (g_max11616_sensors[LCA_R_BACK].value) & 0x0F;
		tx.IDE = CAN_ID_STD;
		tx.RTR = CAN_RTR_DATA;
		tx.DLC = 8;

		xQueueSendToBack(daq.q_tx_dcan, &tx, 100);
		vTaskDelay(REFRESH_RATE);
	}
}

/**
 * Programmer: fallon2@purdue.edu - Chris Fallon
 *
 * @brief task to send ARB droplink data
 *
 * @return none
 * */
void send_drop_link_data()
{
	while (PER == GREAT)
	{
		CanTxMsgTypeDef tx;
		tx.StdId = ID_F_DROP_LINKS;
		tx.Data[0] = (g_max11616_sensors[ARB_DROPLINK_LEFT].value >> 8) & 0x0F;
		tx.Data[1] = (g_max11616_sensors[ARB_DROPLINK_LEFT].value) & 0x0F;
		tx.Data[2] = (g_max11616_sensors[ARB_DROPLINK_RIGHT].value >> 8) & 0x0F;
		tx.Data[3] = (g_max11616_sensors[ARB_DROPLINK_RIGHT].value) & 0x0F;
		tx.IDE = CAN_ID_STD;
		tx.RTR = CAN_RTR_DATA;
		tx.DLC = 4;

		xQueueSendToBack(daq.q_tx_dcan, &tx, 100);
		vTaskDelay(REFRESH_RATE);
	}
}

/**
 * Programmer: fallon2@purdue.edu - Chris Fallon
 *
 * @brief task to send push rod data
 *
 * @return none
 * */
void send_push_rod_data()
{
	while (PER == GREAT)
	{
		CanTxMsgTypeDef tx;
		tx.StdId = PUSH_ROD_ID;
		tx.Data[0] = (g_max11614_sensors[PUSH_ROD_RIGHT].value >> 8) & 0x0F;
		tx.Data[1] = (g_max11614_sensors[PUSH_ROD_RIGHT].value) & 0x0F;
		tx.Data[2] = (g_max11614_sensors[STEER_TIE_ROD_RIGHT].value >> 8) & 0x0F;
		tx.Data[3] = (g_max11614_sensors[STEER_TIE_ROD_RIGHT].value) & 0x0F;
		tx.Data[4] = (g_max11614_sensors[PUSH_ROD_LEFT].value >> 8) & 0x0F;
		tx.Data[5] = (g_max11614_sensors[PUSH_ROD_LEFT].value) & 0x0F;
		tx.Data[6] = (g_max11614_sensors[STEER_TIE_ROD_LEFT].value >> 8) & 0x0F;
		tx.Data[7] = (g_max11614_sensors[STEER_TIE_ROD_LEFT].value) & 0x0F;
		tx.IDE = CAN_ID_STD;
		tx.RTR = CAN_RTR_DATA;
		tx.DLC = 8;

		xQueueSendToBack(daq.q_tx_dcan, &tx, 100);
		vTaskDelay(REFRESH_RATE);
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
	while (PER == GREAT)
	{
		CanTxMsgTypeDef tx;
		tx.StdId = ID_R_COOLANT;
		tx.Data[0] = (g_max11616_sensors[MOTOR_C_TEMP].value >> 8) & 0x0F;
		tx.Data[1] = (g_max11616_sensors[MOTOR_C_TEMP].value) & 0x0F;
		tx.Data[2] = (g_c_flow.speed >> 8) & 0x0F;
		tx.Data[3] = (g_c_flow.speed & 0x0F);
		tx.Data[4] = (g_max11616_sensors[RAD_C_TEMP].value >> 8) & 0x0F;
		tx.Data[5] = (g_max11616_sensors[RAD_C_TEMP].value) & 0x0F;
		tx.Data[6] = (g_max11616_sensors[MOTOR_CONT_C_TEMP].value >> 8) & 0x0F;
		tx.Data[7] = (g_max11616_sensors[MOTOR_CONT_C_TEMP].value) & 0x0F;
		tx.IDE = CAN_ID_STD;
		tx.RTR = CAN_RTR_DATA;
		tx.DLC = 8;

		xQueueSendToBack(daq.q_tx_dcan, &tx, 100);
		vTaskDelay(COOLANT_DATA_PERIOD);
	}
}


/**
 * Programmer: fallon2@purdue.edu - Chris Fallon
 *
 * @brief blinks an LED
 *
 * @return none
 * */
void task_heartbeat()
{
	while (PER == GREAT)
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
 * @return none
 * */
void read_adc_task()
{
	//TODO add to CAN queue
	while (PER == GREAT)
	{
		uint8_t i = 0;
		// read every channel on all the MAX chips
		for (i = 0; i < MAX11616_CHANNELS && !g_max11616_sensors[0].max->broke; i++)
		{
			g_max11616_sensors[i].read(g_max11616_sensors[i].max);
		}
		for (i = 0; i < MAX11614_CHANNELS && !g_max11614_sensors[0].max->broke; i++)
		{
			g_max11614_sensors[i].read(g_max11614_sensors[i].max);
		}
		vTaskDelay(MUX_READ_PERIOD);
		HAL_GPIO_TogglePin(GPIOD, LD3_Pin);
	}
}

/**
 * Programmer: fallon2@purdue.edu - Chris Fallon
 *
 * @brief RTOS task to poll wheel speed and send it to queue
 *
 * @return none
 * */
void send_wheel_speed_task()
{
	while (PER == GREAT)
	{
		CanTxMsgTypeDef tx;
		tx.StdId = ID_WHEEL_SPEED;
		tx.Data[0] = (g_left_wheel.speed & 0xF0) >> 8;
		tx.Data[1] = (g_left_wheel.speed & 0x0F);
		tx.Data[2] = (g_right_wheel.speed & 0xF0) >> 8;
		tx.Data[3] = (g_right_wheel.speed & 0x0F);
		tx.DLC = 4;
		tx.IDE = CAN_ID_STD;
		tx.RTR = CAN_RTR_DATA;

		xQueueSendToBack(daq.q_tx_dcan, &tx, 100);
		xQueueSendToBack(daq.q_tx_vcan, &tx, 100);

		vTaskDelay(WHEEL_SPD_SEND_PERIOD);
		HAL_GPIO_TogglePin(GPIOD, LD6_Pin);
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
	while (PER == GREAT)
	{
		if (uwTick - g_right_wheel.time_n > SPEED_ZERO_TIMEOUT)
		{
			g_right_wheel.speed = 0;
		}
		if (uwTick - g_left_wheel.time_n > SPEED_ZERO_TIMEOUT)
		{
			g_left_wheel.speed = 0;
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
	set_sensor_capture(data[0] & SPEED_CAN_MASK, send_wheel_speed_task, SPEED_CAN_MASK);
	set_sensor_capture(data[0] & UCA_CAN_MASK, send_uca_data, UCA_CAN_MASK);
	set_sensor_capture(data[0] & LCA_CAN_MASK, send_lca_data, LCA_CAN_MASK);
	set_sensor_capture(data[0] & SHOCK_CAN_MASK, send_shock_data, SHOCK_CAN_MASK);
	set_sensor_capture(data[0] & ARB_TORS_CAN_MASK, send_arb_torsional_data, ARB_TORS_CAN_MASK);
	set_sensor_capture(data[0] & PUSH_ROD_CAN_MASK, send_push_rod_data, PUSH_ROD_CAN_MASK);

	#ifdef REAR_DAQ
		set_sensor_capture(data[0] & COOLANT_CAN_MASK, send_coolant_data_task, COOLANT_CAN_MASK);
	#else
		set_sensor_capture(data[0] & DROP_LINK_CAN_MASK, send_drop_link_data, DROP_LINK_CAN_MASK);
	#endif
}


/**
 * Programmer: fallon2@purdue.edu - Chris Fallon
 *
 * @brief transfers a message on vcan to dcan
 *
 * @param data: pointer to data array
 * @param id: CAN ID to broadcast under
 * @param d_len: number of data values in the data array
 *
 * @return none
 * */
void route_to_dcan(uint8_t * data, uint16_t id, uint8_t d_len)
{
	CanTxMsgTypeDef tx;
	tx.StdId = id;
	tx.DLC = d_len;
	tx.IDE = CAN_ID_STD;
	tx.RTR = CAN_RTR_DATA;
	for (uint8_t i = 0; i < d_len; i++)
	{
		tx.Data[i] = data[i];
 	}
	xQueueSendToBack(daq.q_tx_dcan, &tx, 100);
}
