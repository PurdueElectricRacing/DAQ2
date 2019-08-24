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
#include "DAQ_CAN.h"
#include "maxsensor.h"

#define GREAT 1
#define PER GREAT

#define REAR_DAQ		// undefine this if this is the front DAQ board
//#define STRAIN_GAUGES

#ifdef REAR_DAQ
	#define ID_WHEEL_SPEED   ID_R_WHEEL_SPEED
	#define SHOCK_POT_ID     ID_R_SHOCKS
	#define LCA_ID           ID_R_LCA
	#define UCA_ID           ID_R_UCA
	#define ARB_TORSIONAL_ID ID_R_ARB
	#define TIRE_TEMP_ID     ID_R_TIRE_TEMP
	#define PUSH_ROD_ID      ID_R_TIE_PUSH
  #define ERROR_ID         REAR_ERROR
#else
 	#define ID_WHEEL_SPEED   ID_F_WHEEL_SPEED
	#define SHOCK_POT_ID     ID_F_SHOCKS
	#define LCA_ID           ID_F_LCA
	#define UCA_ID           ID_F_UCA
	#define ARB_TORSIONAL_ID ID_F_ARB
	#define TIRE_TEMP_IP     ID_F_TIRE_TEMP
	#define PUSH_ROD_ID      ID_F_STEER_PUSH
  #define ERROR_ID         FRONT_ERROR
#endif

// mask bits for parsing the CAN toggle message
#define SPEED_CAN_MASK     0b10000000
#define UCA_CAN_MASK       0b01000000
#define LCA_CAN_MASK       0b00100000
#define SHOCK_CAN_MASK     0b00010000
#define ARB_TORS_CAN_MASK  0b00001000
#define PUSH_ROD_CAN_MASK  0b00000100
#define COOLANT_CAN_MASK   0b00000010
#define DROP_LINK_CAN_MASK 0b00000001

// enable all by default
#define ENABLE_CAN_TX 0xFF

#define MUX_READ_PERIOD       100 	// 100hz
#define HEARTBEAT_PERIOD      1000	// 1hz
#define WHEEL_SPD_SEND_PERIOD 100  // 100hz
#define COOLANT_DATA_PERIOD   100
#define SPEED_ZERO_TIMEOUT    100	// 1hz
#define SPEED_ZERO_PERIOD		  250	// .25hz
#define REFRESH_RATE          100
#define ERROR_MSG_PERIOD      100

#define ADC_READ_STACK       128
#define DAQ_TASK_STACK       128
#define CAN_TX_TASK_STACK    128


// defines match both the location in the array and also the channel on each MAX chip
// MAX11616 DEVICES BELOW
#define UCA_R_BACK   0
#define UCA_R_FRONT  1
#define LCA_R_FRONT  3
#define LCA_R_BACK   2

#define MOTOR_C_TEMP  		 4
#define RAD_C_TEMP    		 5
#define MOTOR_CONT_C_TEMP  6

#define LEFT_SHOCK_POT   7

#define ARB_DROPLINK_LEFT  8
#define ARB_DROPLINK_RIGHT 9
#define ARB_TORSIONAL      10

#define RIGHT_SHOCK_POT  11

// MAX11614 DEVICES BELOW
#define STEER_TIE_ROD_RIGHT 0
#define STEER_TIE_ROD_LEFT  1

#define PUSH_ROD_RIGHT  2
#define PUSH_ROD_LEFT   3

#define UCA_L_FRONT  4
#define UCA_L_BACK   5
#define LCA_L_FRONT  6
#define LCA_L_BACK   7

// error indexes in the error array
#define L_WHEEL_SPD_ERROR 0
#define R_WHEEL_SPD_ERROR 1
#define MAX11614_ERROR    2
#define MAX11616_ERROR    3
#ifdef REAR_DAQ
	#define ERROR_ID REAR_ERROR
	#define COOL_SPD_ERROR    4
#else
	#define ERROR_ID FRONT_ERROR
#endif

#define EXTRACT_MSB(data) ((data >> 8) & 0xFF)
#define EXTRACT_LSB(data) ((data) & 0xFF)

typedef struct DAQ_t
{
	CAN_HandleTypeDef * vcan;
	CAN_HandleTypeDef * dcan;

	QueueHandle_t			q_rx_dcan;
	QueueHandle_t			q_tx_dcan;
	QueueHandle_t			q_rx_vcan;
	QueueHandle_t			q_tx_vcan;

	uint8_t can_enable;
}DAQ_t;

enum task_t
{
	adc_task          = 0,
	uca_task          = 1,
	lca_task          = 2,
	shock_task        = 3,
	arb_tors_task     = 4,
	push_rod_task     = 5,
	wheel_speed_task  = 6,
  speed_0_task      = 7,
	cool_task         = 8,
	drop_link_task    = 8,
};

enum rinehart_ids
{
  TEMP1 = 0x0A0,
	TEMP2 = 0x0A1,
	TEMP3 = 0x0A2,
	ANALOG_INPUT_VOLTAGES = 0x0A3,
	DIGITAL_INPUT_STATUS  = 0x0A4,
	MOTOR_POSITION_INFO   = 0x0A5,
	CURRENT_INFO = 0x0A6,
	VOLTAGE_INFO = 0x0A7,
	FLUX_INFO    = 0x0A8,
  INTERNAL_VOLTAGES = 0x0A9,
	INTERNAL_STATES   = 0x0AA,
	FAULT_CODES       = 0x0AB,
	TORQUE_TIMER_INFO = 0x0AC,
	MOD_INDEX_FLUX_WEAK_OUT_INFO = 0x0AD,
  FIRMWARE_INFO = 0x0AE,
} rinehart_ids;

#define NUM_TASKS 9

void send_heartbeat(uint8_t module);

void init_daq2(volatile DAQ_t * controller, I2C_HandleTypeDef * hi2c, I2C_HandleTypeDef * hi2c1, TIM_HandleTypeDef * htim, CAN_HandleTypeDef * vcan, CAN_HandleTypeDef * dcan);
void start_daq2();
void init_max_arrays();

void task_heartbeat();

void read_adc_task();
void process_sensor_enable(uint8_t * data);
void wheel_speed_zero_task();
void set_wheel_speed_capture(uint8_t enable);

void send_wheel_speed_task();
void send_coolant_data_task();
void send_shock_data();
void send_arb_torsional_data();
void send_uca_data();
void send_lca_data();
void send_drop_link_data();
void send_push_rod_data();
void send_tire_temp_data();
void set_sensor_capture(uint8_t enable, TaskHandle_t function, uint8_t mask);
void error_task();
void route_to_dcan(uint8_t * data, uint16_t id, uint8_t d_len);

#endif /* DAQ2_H_ */
