/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    can.c
 * @brief   This file provides code for the configuration
 *          of the CAN instances.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

CAN_HandleTypeDef hcan2;

/* CAN2 init function */
void MX_CAN2_Init(void) {

	/* USER CODE BEGIN CAN2_Init 0 */

	/* USER CODE END CAN2_Init 0 */

	/* USER CODE BEGIN CAN2_Init 1 */

	/* USER CODE END CAN2_Init 1 */
	hcan2.Instance = CAN2;
	hcan2.Init.Prescaler = 9;
	hcan2.Init.Mode = CAN_MODE_NORMAL;
	hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan2.Init.TimeSeg1 = CAN_BS1_3TQ;
	hcan2.Init.TimeSeg2 = CAN_BS2_4TQ;
	hcan2.Init.TimeTriggeredMode = DISABLE;
	hcan2.Init.AutoBusOff = DISABLE;
	hcan2.Init.AutoWakeUp = DISABLE;
	hcan2.Init.AutoRetransmission = DISABLE;
	hcan2.Init.ReceiveFifoLocked = DISABLE;
	hcan2.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN2_Init 2 */

	/* USER CODE END CAN2_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef *canHandle) {

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	if (canHandle->Instance == CAN2) {
		/* USER CODE BEGIN CAN2_MspInit 0 */

		/* USER CODE END CAN2_MspInit 0 */
		/* CAN2 clock enable */
		__HAL_RCC_CAN2_CLK_ENABLE();
		__HAL_RCC_CAN1_CLK_ENABLE();

		__HAL_RCC_GPIOB_CLK_ENABLE();
		/**CAN2 GPIO Configuration
		 PB12     ------> CAN2_RX
		 PB13     ------> CAN2_TX
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_12;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_13;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* USER CODE BEGIN CAN2_MspInit 1 */

		/* USER CODE END CAN2_MspInit 1 */
	}
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef *canHandle) {

	if (canHandle->Instance == CAN2) {
		/* USER CODE BEGIN CAN2_MspDeInit 0 */

		/* USER CODE END CAN2_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_CAN2_CLK_DISABLE();
		__HAL_RCC_CAN1_CLK_DISABLE();

		/**CAN2 GPIO Configuration
		 PB12     ------> CAN2_RX
		 PB13     ------> CAN2_TX
		 */
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12 | GPIO_PIN_13);

		/* USER CODE BEGIN CAN2_MspDeInit 1 */

		/* USER CODE END CAN2_MspDeInit 1 */
	}
}

/* USER CODE BEGIN 1 */

HAL_StatusTypeDef CAN_Start() {
	return HAL_CAN_Start(&hcan2);
}

HAL_StatusTypeDef CAN_Activate() {
	return HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

HAL_StatusTypeDef CAN_Send(struct CANMessage *ptr) {
	return HAL_CAN_AddTxMessage(&hcan2, &ptr->TxHeader, (uint8_t*) ptr->data,
			&ptr->TxMailbox);
}

void CAN_SettingsInit(struct CANMessage *ptr) {
	CAN_Start();
	CAN_Activate();
	ptr->TxHeader.IDE = CAN_ID_STD;
	ptr->TxHeader.StdId = 0x00;
	ptr->TxHeader.RTR = CAN_RTR_DATA;
	ptr->TxHeader.DLC = 8;
}

void setCANId(struct CANMessage *ptr, uint32_t id) {
	ptr->TxHeader.StdId = id;
}

void CAN_Send_Voltage(struct CANMessage *ptr, uint16_t *read_volt) {
	uint16_t CAN_ID = 0x630;
	setCANId(ptr, CAN_ID);
	for (int i = 0; i < NUM_CELLS; i++) {
		if (i % 4 == 0) {
			uint8_t temp_volt = i;
			ptr->data[0] = read_volt[temp_volt];
			ptr->data[1] = read_volt[temp_volt] >> 8;
			temp_volt += 1;
			ptr->data[2] = read_volt[temp_volt];
			ptr->data[3] = read_volt[temp_volt] >> 8;
			temp_volt += 1;
			ptr->data[4] = read_volt[temp_volt];
			ptr->data[5] = read_volt[temp_volt] >> 8;
			temp_volt += 1;
			ptr->data[6] = read_volt[temp_volt];
			ptr->data[7] = read_volt[temp_volt] >> 8;
		}
		if (i > 0 && i % 4 == 0) {
			CAN_ID = CAN_ID + 0x01;
			setCANId(ptr, CAN_ID);
		}
		HAL_Delay(10);
		CAN_Send(ptr);
	}

}

void CAN_Send_Temperature(struct CANMessage *ptr, uint16_t *read_temp) {
	uint16_t CAN_ID = 0x680;
	setCANId(ptr, CAN_ID);
	for (uint8_t i = 0; i < NUM_THERM_TOTAL; i++) {
		if (i % 4 == 0) {
			uint8_t temp_volt = i;
			ptr->data[0] = read_temp[temp_volt];
			ptr->data[1] = read_temp[temp_volt] >> 8;
			temp_volt += 1;
			ptr->data[2] = read_temp[temp_volt];
			ptr->data[3] = read_temp[temp_volt] >> 8;
			temp_volt += 1;
			ptr->data[4] = read_temp[temp_volt];
			ptr->data[5] = read_temp[temp_volt] >> 8;
			temp_volt += 1;
			ptr->data[6] = read_temp[temp_volt];
			ptr->data[7] = read_temp[temp_volt] >> 8;
		}
		if (i > 0 && i % 4 == 0) {
			CAN_ID = CAN_ID + 0x01;
			setCANId(ptr, CAN_ID);
		}
		HAL_Delay(10);
		CAN_Send(ptr);
	}

}

void CAN_Send_Cell_Summary(struct CANMessage *ptr,
		struct batteryModuleVoltage *batt) {
	uint16_t CAN_ID = 0x622;
	setCANId(ptr, CAN_ID);

	ptr->data[0] = batt->cell_temp_highest;
	ptr->data[1] = (batt->cell_volt_highest) >> 8;
	ptr->data[2] = batt->cell_volt_lowest;
	ptr->data[3] = (batt->cell_volt_lowest) >> 8;
	ptr->data[4] = batt->cell_temp_highest;
	ptr->data[5] = (batt->cell_temp_highest) >> 8;
	ptr->data[6] = batt->cell_temp_lowest;
	ptr->data[7] = (batt->cell_temp_lowest) >> 8;

	HAL_Delay(10);
	CAN_Send(ptr);
}

void CAN_Send_Safety_Checker(struct CANMessage *ptr, uint8_t* faults, uint8_t* warnings){
	uint16_t CAN_ID = 0x600;
	setCANId(ptr, CAN_ID);
	ptr->data[0] = *faults & 0xFF;
	ptr->data[1] = *warnings & 0xFF;
	HAL_Delay(10);
	CAN_Send(ptr);
}
/* USER CODE END 1 */
