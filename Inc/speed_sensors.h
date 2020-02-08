#ifndef SPEED_SENSOR_H
#define SPEED_SENSOR_H


#include "main.h"
#include "stm32f4xx_hal.h"
#include "daq2.h"

#define C_FLOW_CHANNEL  TIM_CHANNEL_4
#define PULSE_PER_LITER 450
#define ML_PER_PULSE    (1.0f / 450.0f)
#define LPM_CONVERSION  7.5f
#define FLOW_ZERO_DT    75

#ifdef REAR_DAQ
  #define DEGREES_PER_COUNT (360.0f / 1024.0f)
#else
  #define DEGREES_PER_COUNT (360.0f / 720.0f)
#endif

#define MILLIS_TO_MINUTES 60 * 1000
#define FULL_CIRCLE       360
#define WHEEL_ZERO_DT     200

#define LEFT_WHEEL_LED 	LD6_Pin
#define RIGHT_WHEEL_LED	LD3_Pin
#define LED_Port		GPIOD

#define SCALAR  10000

typedef enum
{
  NO_ERROR = 0,
  AMPLITUDE_ERROR = 1,
  FREQUENCY_ERROR = 2,
  OTHER_ERROR = 3,
} encoder_error_t;


typedef struct
{
  TIM_HandleTypeDef * htim;
  encoder_error_t error;
  uint16_t        a_time;
  uint16_t        a_time_minus_1;
  uint32_t        channel;
  uint32_t        speed;
  
} encoder_t;



typedef struct
{
	TIM_HandleTypeDef * htim;
	uint32_t 			time_n;
	uint32_t 			time_n_minus_1;
	uint32_t			speed;
	uint8_t       error;
  uint32_t      channel;
} hall_sensor;

enum HALL_SENSOR
{
	RIGHT_WHEEL,
	LEFT_WHEEL,
	C_FLOW,
}HALL_SENSOR;

void init_encoder(encoder_t * enc, TIM_HandleTypeDef * htim, uint32_t channel);
void determine_error(encoder_t * enc);

void store_enc_times(encoder_t * enc);

void set_zero(hall_sensor * sensor, uint32_t zero_time);
void init_hall_sensor(volatile hall_sensor * sensor, TIM_HandleTypeDef *htim, uint32_t channel);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void store_hall_times(volatile hall_sensor *sensor);
uint32_t calculate_wheel_speed(uint32_t dt);
uint32_t calculate_flow_rate(uint32_t dt);



//The sensor comes with three wires: red (5-24VDC power), black (ground)
//and yellow (Hall effect pulse output). By counting the pulses from the
//output of the sensor, you can easily track fluid movement: each pulse
//is approximately 2.222222 milliliters. Note this isn't a precision sensor,
//and the pulse rate does vary a bit depending on the flow rate, fluid
//pressure and sensor orientation.

//Working Voltage: 5 to 18VDC
//Max current draw: 15mA @ 5V
//Working Flow Rate: 1 to 30 Liters/Minute
//Working Temperature range: -25 to 80�C
//Working Humidity Range: 35%-80% RH
//Maximum water pressure: 2.0 MPa
//Output duty cycle: 50% +-10%
//Output rise time: 0.04us
//Output fall time: 0.18us
//Flow rate pulse characteristics: Frequency (Hz) = 7.5 * Flow rate (L/min)
//Pulses per Liter: 450
//Durability: minimum 300,000 cycles


#endif
