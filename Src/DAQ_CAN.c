/*
 * DAQ_CAN.c
 *
 *  Created on: Jan 26, 2019
 *      Author: Chris
 */

#include "DAQ_CAN.h"

extern volatile DAQ_t daq;

/***************************************************************************
*
*     Function Information
*
*     Name of Function: HAL_CAN_RxFifo0MsgPendingCallback
*
*     Programmer's Name: Ben Ng, xbenng@gmail.com
*
*     Function Return Type:
*
*     Parameters (list data type, name, and comment one per line):
*
*     Global Dependents:
*	  1. ;
*
*     Function Description:
*			To be called by HAL_CAN_IRQHandler when a CAN message is received.
*
***************************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	//HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);

	CanRxMsgTypeDef rx;
	CAN_RxHeaderTypeDef header;
	HAL_CAN_GetRxMessage(hcan, 0, &header, rx.Data);
	rx.DLC = header.DLC;
	rx.StdId = header.StdId;
	xQueueSendFromISR(daq.q_rx_dcan, &rx, NULL);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CanRxMsgTypeDef rx;
	CAN_RxHeaderTypeDef header;
	HAL_CAN_GetRxMessage(hcan, 1, &header, rx.Data);
	rx.DLC = header.DLC;
	rx.StdId = header.StdId;
	xQueueSendFromISR(daq.q_rx_vcan, &rx, NULL);
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: RXCANProcessTask
*
*     Programmer's Name: Ben Ng, xbenng@gmail.com
*
*     Function Return Type:
*
*     Parameters (list data type, name, and comment one per line):
*       1. CAN_HandleTypeDef *hcan, hcan structure address to add filter to
*
*     Global Dependents:
*	    1.
*
*     Function Description: Filter Configuration.
*
***************************************************************************/
void DCANFilterConfig(CAN_HandleTypeDef * hcan)
{
	  CAN_FilterTypeDef FilterConf;
	  FilterConf.FilterIdHigh =         ID_PEDALBOX2 << 5; // 2 num
	  FilterConf.FilterIdLow =          ID_DASHBOARD << 5; // 0
	  FilterConf.FilterMaskIdHigh =     0x7ff;       // 3
	  FilterConf.FilterMaskIdLow =      0x7fe;       // 1
	  FilterConf.FilterFIFOAssignment = CAN_FilterFIFO0;
	  FilterConf.FilterBank = 0;
	  FilterConf.FilterMode = CAN_FILTERMODE_IDLIST;
	  FilterConf.FilterScale = CAN_FILTERSCALE_16BIT;
	  FilterConf.FilterActivation = ENABLE;
	  HAL_CAN_ConfigFilter(hcan, &FilterConf);
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: RXCANProcessTask
*
*     Programmer's Name: Ben Ng, xbenng@gmail.com
*
*     Function Return Type:
*
*     Parameters (list data type, name, and comment one per line):
*       1. CAN_HandleTypeDef *hcan, hcan structure address to add filter to
*
*     Global Dependents:
*	    1.
*
*     Function Description: Filter Configuration.
*
***************************************************************************/
void VCANFilterConfig(CAN_HandleTypeDef * hcan)
{
	  CAN_FilterTypeDef FilterConf;
	  FilterConf.FilterIdHigh =         ID_DASHBOARD << 5; // 2 num
	  FilterConf.FilterIdLow =          ID_DASHBOARD << 5; // 0
	  FilterConf.FilterMaskIdHigh =     0x7FF;       // 3
	  FilterConf.FilterMaskIdLow =      0x7FF;       // 1
	  FilterConf.FilterFIFOAssignment = CAN_FilterFIFO1;
	  FilterConf.FilterBank = 1;
	  FilterConf.FilterMode = CAN_FILTERMODE_IDLIST;
	  FilterConf.FilterScale = CAN_FILTERSCALE_16BIT;
	  FilterConf.FilterActivation = ENABLE;
	  HAL_CAN_ConfigFilter(hcan, &FilterConf);
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: taskTX_DCAN
*
*     Programmer's Name: Ben Ng, xbenng@gmail.com
*
*     Function Return Type: none
*
*     Parameters (list data type, name, and comment one per line):
*
*      Global Dependents:
*	   1.
*
*     Function Description:
*     	Task function to send CAN messages using the CAN peripheral
*
***************************************************************************/
void taskTX_DCAN()
{
	CanTxMsgTypeDef tx;

	for (;;)
	{
		//check if this task is triggered
		if (xQueuePeek(daq.q_tx_dcan, &tx, portMAX_DELAY) == pdTRUE)
		{
			xQueueReceive(daq.q_tx_dcan, &tx, portMAX_DELAY);  //actually take item out of queue
			CAN_TxHeaderTypeDef header;
			header.DLC = tx.DLC;
			header.IDE = tx.IDE;
			header.RTR = tx.RTR;
			header.StdId = tx.StdId;
			header.TransmitGlobalTime = DISABLE;
			uint32_t mailbox;
			while (!HAL_CAN_GetTxMailboxesFreeLevel(daq.dcan)); // while mailboxes not free
			HAL_CAN_AddTxMessage(daq.dcan, &header, tx.Data, &mailbox);
			HAL_GPIO_TogglePin(GPIOD, LD4_Pin);
		}
	}
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: taskTX_VCAN
*
*     Programmer's Name: Ben Ng, xbenng@gmail.com
*
*     Function Return Type: none
*
*     Parameters (list data type, name, and comment one per line):
*
*      Global Dependents:
*	   1.
*
*     Function Description:
*     	Task function to send CAN messages using the CAN peripheral
*
***************************************************************************/
void taskTX_VCAN()
{
	CanTxMsgTypeDef tx;

	for (;;)
	{
		//check if this task is triggered
		if (xQueuePeek(daq.q_tx_vcan, &tx, portMAX_DELAY) == pdTRUE)
		{
			xQueueReceive(daq.q_tx_vcan, &tx, portMAX_DELAY);  //actually take item out of queue
			CAN_TxHeaderTypeDef header;
			header.DLC = tx.DLC;
			header.IDE = tx.IDE;
			header.RTR = tx.RTR;
			header.StdId = tx.StdId;
			header.TransmitGlobalTime = DISABLE;
			uint32_t mailbox;
			while (!HAL_CAN_GetTxMailboxesFreeLevel(daq.vcan)); // while mailboxes not free
			HAL_CAN_AddTxMessage(daq.vcan, &header, tx.Data, &mailbox);
		}
	}
}

/***************************************************************************
*
*     Function Information
*
*     Name of Function: taskRXCANprocess
*
*     Programmer's Name: Ben Ng, xbenng@gmail.com
*
*     Function Return Type: none
*
*     Parameters (list data type, name, and comment one per line):
*
*      Global Dependents:
*	   1.-
*
*     Function Description:
*     	Task function to process received CAN Messages.
*     	CanRxMsgTypeDef are sent here to the q_rxcan queue to be processed
*     	from the CAN RX interrupt handler.
*     	The data is process and handled according to what kind of message is received
*
***************************************************************************/
void taskRX_DCANProcess()
{

}

void taskRX_VCANProcess()
{
	CanRxMsgTypeDef rx;  //CanRxMsgTypeDef to be received on the queue

	for(;;)
	{
		//if there is a CanRxMsgTypeDef in the queue, pop it, and store in rx
		if (xQueueReceive(daq.q_rx_vcan, &rx, portMAX_DELAY) == pdTRUE)
		{
			HAL_GPIO_TogglePin(LD4_GPIO_Port, LD3_Pin);
			//A CAN message has been recieved
			//check what kind of message we received
			switch (rx.StdId)
			{
				case	ID_ENABLE_DAQ:
				{
					process_sensor_enable(rx.Data);
					break;
				}

			}

		}
	}
}




