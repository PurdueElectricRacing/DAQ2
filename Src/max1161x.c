/**
 * @note       STILL IN DEVELOPMENT
 * @file       max1161x.c
 * @author     Simon Burkhardt github.com/mnemocron
 * @copyright  MIT license
 * @date       06.2018
 * @brief      C library for the max1161xEEE+ I2C ADC for STM32 HAL.
 * @details
 * @see        github.com/mnemocron
 * @see        https://datasheets.maximintegrated.com/en/ds/MAX11612-MAX11617.pdf
 * @see        https://github.com/AllAboutEE/MAX11609EEE-Breakout-Board/tree/master/Software/Arduino/AllAboutEE-MAX11609-Library
 */
#include <max1161x.h>
#include "main.h"

/**
 * @brief      writes a single value into a max1161x register
 * @param      *mux, pointer to the max1161x typedef struct
 * @param      reg, the destination register's address
 * @param      val, the value for the destination register
 */
uint8_t max1161x_Write8(max1161x *mux, uint8_t reg, uint8_t val){
 	if(HAL_I2C_Mem_Write(mux->wireIface, mux->devAddress, reg, 1, &val, 1, 10) != HAL_OK)
		return 1;
	return 0;
}

/**
 * @brief       reads a single value from a max1161x register
 * @param       *mux, pointer to the max1161x typedef struct
 * @param       reg, the destination register's address
 * @param       val, pointer to the location where the value shall be stored
 * @return      0 on success, 1 on transmission error
 */
uint8_t max1161x_Read8(max1161x *mux, uint8_t reg, uint8_t *val){
	if(HAL_I2C_Mem_Read(mux->wireIface, mux->devAddress, reg, 1, val, 1, 10) != HAL_OK)
		return 1;
	return 0;
}

/**
 * @brief        sets up the max1161x mux with the provided Vref setting
 * @param        *mux, pointer to the max1161x typedef struct
 * @param        *wireIface a pointer to a HAL I2C_HandleTypeDef
 * @param 		 address of the mux on the I2C bus
 */
void max1161x_Init(max1161x* mux, I2C_HandleTypeDef* wireIface, uint16_t address, uint8_t vRef){
	mux->wireIface = wireIface;
	mux->devAddress = address;

	if (HAL_I2C_IsDeviceReady(wireIface, mux->devAddress, 2, 100) != HAL_OK)
	{
		mux->broke++;
	}
	// 0 - don't care
	// 1 - reset configuration register to default
	// 2 - unipolar
	// 3 - internal clock
	// 4 - SEL0 (vRef)
	// 5 - SEL1 (vRef)
	// 6 - SEL2 (vRef)
	vRef = (vRef<<4) & 0xf0;
	vRef |= 2; // do not reset the setup register
	mux->broke += max1161x_Setup(mux, vRef);
	// 0 - Single Ended
	// 1 to 4 - Channel Select:  7
	// 5 to 6 - Scan Mode: read channels up to selected channel
	uint8_t config_byte = 0x00;
	config_byte |= 1; // single ended mode
	config_byte |= ((1<<5) & (1<<6)); // SCAN bits: convert only the channel selected by CS bits
	mux->broke += max1161x_Configuration(mux, config_byte);
}

uint8_t max1161x_Setup(max1161x* mux, uint8_t data){
	data = data | 0x80; // make REG bit 7 = 1 (setup byte)
	if(HAL_I2C_Master_Transmit(mux->wireIface, mux->devAddress, &data, 1, 10) != HAL_OK)
		return 1;
	return 0;
}

uint8_t max1161x_Configuration(max1161x* mux, uint8_t data){
	data = data & (~0x80); // make REG bit 7 = 0 (configuration byte)
	if(HAL_I2C_Master_Transmit(mux->wireIface, mux->devAddress, &data, 1, 10) != HAL_OK)
		return 1;
	return 0;
}

/**
 * Reads one channel.
 * @author Miguel  (5/24/2015)
 * @param channel  The channel to convert or read. Alternatively
 *                 if a channel was set already leave null.
 * @param val      Pointer to where the return value should be stored.
 * @return uint8_t
 */
uint8_t max1161x_ADC_Read(max1161x* mux, uint8_t channel, uint16_t* val){
	uint8_t result[2] = {0,0};

	uint8_t configurationByte = ( (channel<<1) & 0x0e) | 0x61;
	max1161x_Configuration(mux, configurationByte);
	// the conversion consists of two bytes per channel
	if(HAL_I2C_Master_Receive(mux->wireIface, mux->devAddress, &result[0], 2, 10) != HAL_OK){
		return 1;
	}
	uint16_t value = 0;
	// cast to uint16_t is necessary to not loose the values by the left shift
	value =  (((uint16_t)result[0] & 0x000f) << 8); // MSB is returned first
	value += ((uint16_t)result[1] & 0x00ff); // read LSB
	*val = value;
	return 0;
}

/**
 * Reads all channels conversion into a buffer/array.
 *
 * @author Miguel (5/24/2015)
 *
 * @param buffer an array where the channel read values are put.
 */
uint8_t max1161x_Scan(max1161x* mux, uint16_t* buffer){
	uint8_t ret = 0;
	uint8_t configurationByte = 0xf0;
	uint8_t channels;

	if (mux->devAddress == MAX11614_ADDR)
	{
		channels = MAX11614_CHANNELS;
	}
	else
	{
		channels = MAX11616_CHANNELS;
	}

	ret += max1161x_Configuration(mux, configurationByte);
	// 2 bytes per channel. There are 8 channels
	for(uint8_t i = 0; i < channels; i++){
		ret += max1161x_ADC_Read(mux, i, &buffer[i]);
	}
	return ret;
}
