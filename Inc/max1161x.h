
/**
  * @file       max1161x.h
  * @author     Simon Burkhardt github.com/mnemocron
  * @copyright  MIT license
  * @date       06.2018
  * @brief      C library for the max1161xEEE+ I2C ADC for STM32 HAL.
  * @details
  * @see 	      github.com/mnemocron
  * @see 	      https://datasheets.maximintegrated.com/en/ds/MAX11612-MAX11617.pdf
  * @see        https://github.com/AllAboutEE/MAX11609EEE-Breakout-Board/tree/master/Software/Arduino/AllAboutEE-MAX11609-Library
  */

#ifndef MAX1161X_H_
#define MAX1161X_H_

/**
  * @note 		tested using STM32F373
  */
#include "main.h"
#include "stm32f4xx_hal.h"

/** @see Datasheet p.19 Table 6
  */

#define MAX11614_ADDR  0x66
#define MAX11616_ADDR  0x6A

#define MAX11614_CHANNELS  12
#define MAX11616_CHANNELS  8

#define max1161x_REF_VDD      0x00
#define max1161x_REF_EXTERNAL 0x02
#define max1161x_REF_INTERNAL 0x04
#define max1161x_ANANLOG_IN   0x00
#define max1161x_REF_OUT      0x02
#define max1161x_INT_REF_ON   0x01

typedef struct
{
	uint16_t devAddress;
	I2C_HandleTypeDef *wireIface;
	uint8_t broke;
	uint8_t enable;
	uint16_t data[MAX11614_CHANNELS];
} max1161x;

max1161x new_max1161x              (void);
void    max1161x_Init              (max1161x*, I2C_HandleTypeDef*, uint16_t, uint8_t);

uint8_t max1161x_Write8            (max1161x*, uint8_t, uint8_t);
uint8_t max1161x_Read8             (max1161x*, uint8_t, uint8_t*);
uint8_t max1161x_Setup             (max1161x*, uint8_t);
uint8_t max1161x_Configuration 	   (max1161x*, uint8_t);
uint8_t max1161x_ADC_Read          (max1161x*, uint8_t, uint16_t*);
uint8_t max1161x_Scan              (max1161x*, uint16_t*);

#endif

