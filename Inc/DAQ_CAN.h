/*
 * DAQ_CAN.h
 *
 *  Created on: Jan 26, 2019
 *      Author: Chris
 */

#ifndef DAQ_CAN_H_
#define DAQ_CAN_H_

#include "daq2.h"
#define ID_PEDALBOX1						0x500
#define ID_PEDALBOX2						0x501

// IDs for wheel speed sensors
#define ID_F_WHEEL_SPEED				0x700
#define ID_R_WHEEL_SPEED				0x701

// IDs for tire temps sensors
#define ID_F_TIRE_TEMP					0x710
#define ID_R_TIRE_TEMP					0x711

#define ID_R_COOLANT						0x720
#define ID_R_COOLANT_SPD        0x721

// IDs for shock pots
#define ID_F_SHOCKS							0x730
#define ID_R_SHOCKS							0x731

#define ID_F_DROP_LINKS					0x740

#define ID_F_STEER_PUSH					0x750
#define ID_R_TIE_PUSH						0x760

#define ID_F_LCA								0x770
#define ID_R_LCA								0x780
#define ID_F_UCA								0x790
#define ID_R_UCA								0x7A0

#define ID_F_ARB								0x7B0
#define ID_R_ARB								0x7B1

#define ID_DASHBOARD						0x7C0
#define DASHBOARD_LENGTH        4

#define ID_F_IMU								0x7D0

#define ID_ENABLE_DAQ						0x7E0

#define FRONT_ERROR             0x7F0
#define REAR_ERROR              0x7F1

#define QUEUE_SIZE_RXCAN_1			16
#define QUEUE_SIZE_RXCAN_2			16
#define QUEUE_SIZE_TXCAN_1			10
#define QUEUE_SIZE_TXCAN_2			10



/**
  * @brief  CAN Tx message structure definition
  */
typedef struct
{
  uint32_t StdId;    /*!< Specifies the standard identifier.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF */

  uint32_t ExtId;    /*!< Specifies the extended identifier.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 0x1FFFFFFF */

  uint32_t IDE;      /*!< Specifies the type of identifier for the message that will be transmitted.
                          This parameter can be a value of @ref CAN_Identifier_Type */

  uint32_t RTR;      /*!< Specifies the type of frame for the message that will be transmitted.
                          This parameter can be a value of @ref CAN_remote_transmission_request */

  uint32_t DLC;      /*!< Specifies the length of the frame that will be transmitted.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 8 */

  uint8_t Data[8];   /*!< Contains the data to be transmitted.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 0xFF */

}CanTxMsgTypeDef;

/**
  * @brief  CAN Rx message structure definition
  */
typedef struct
{
  uint32_t StdId;       /*!< Specifies the standard identifier.
                             This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF */

  uint32_t ExtId;       /*!< Specifies the extended identifier.
                             This parameter must be a number between Min_Data = 0 and Max_Data = 0x1FFFFFFF */

  uint32_t IDE;         /*!< Specifies the type of identifier for the message that will be received.
                             This parameter can be a value of @ref CAN_Identifier_Type */

  uint32_t RTR;         /*!< Specifies the type of frame for the received message.
                             This parameter can be a value of @ref CAN_remote_transmission_request */

  uint32_t DLC;         /*!< Specifies the length of the frame that will be received.
                             This parameter must be a number between Min_Data = 0 and Max_Data = 8 */

  uint8_t Data[8];      /*!< Contains the data to be received.
                             This parameter must be a number between Min_Data = 0 and Max_Data = 0xFF */

  uint32_t FMI;         /*!< Specifies the index of the filter the message stored in the mailbox passes through.
                             This parameter must be a number between Min_Data = 0 and Max_Data = 0xFF */

  uint32_t FIFONumber;  /*!< Specifies the receive FIFO number.
                             This parameter can be CAN_FIFO0 or CAN_FIFO1 */

}CanRxMsgTypeDef;

void ISR_RXCAN();
void DCANFilterConfig();
void VCANFilterConfig();
void taskRX_DCANProcess();
void taskRX_VCANProcess();
void taskTX_DCAN();
void taskTX_VCAN();
void taskRXCAN();
void process_mux_data();
void process_wheel_speed();


#endif /* DAQ_CAN_H_ */
