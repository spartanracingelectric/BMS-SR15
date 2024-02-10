/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

struct batteryModuleVoltage {
	uint16_t *cell_volt;
	uint16_t *cell_temp;
	uint16_t *cell_volt_lowest;
	uint16_t *cell_volt_highest;
	uint16_t *cell_temp_lowest;
	uint16_t *cell_temp_highest;
	uint16_t *pack_voltage;
	uint16_t *read_auxreg;

};

struct CANMessage{
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;
	uint8_t data[8];
};


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MCU_USB_INT_Pin GPIO_PIN_0
#define MCU_USB_INT_GPIO_Port GPIOA
#define LTC_nCS_Pin GPIO_PIN_4
#define LTC_nCS_GPIO_Port GPIOA
#define MCU_HEARTBEAT_LED_Pin GPIO_PIN_6
#define MCU_HEARTBEAT_LED_GPIO_Port GPIOC
#define PMUX_ST_Pin GPIO_PIN_8
#define PMUX_ST_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */
#define NUM_DEVICES				1	//1 slave board
#define NUM_CELL_SERIES_GROUP	12	//1 slave board
#define NUM_CELLS				NUM_DEVICES*NUM_CELL_SERIES_GROUP	//multiple slave board
#define NUM_THERM_PER_MOD		12
#define NUM_THERM_TOTAL			NUM_DEVICES*NUM_THERM_PER_MOD
#define NUM_AUX_GROUP			6
#define NUM_AUXES				NUM_DEVICES*NUM_AUX_GROUP
#define LTC_DELAY1				1000 //500ms update delay
#define LTC_DELAY2				0 //500ms update delay
#define CAN1_DELAY				100
#define LED_HEARTBEAT_DELAY_MS	500  //500ms update delay
#define LTC_CMD_RDSTATA			0x0010 //Read status register group A

#define ntcNominal 50000.0f
#define ntcSeriesResistance 50000.0f
#define ntcBetaFactor 3380.0f
#define ntcNominalTemp 25.0f

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
