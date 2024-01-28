/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "6811.h"
#include "print.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_DEVICES				1	//1 slave board
#define NUM_SERIES_GROUP		12	//1 slave board
#define NUM_CELLS				NUM_DEVICES*NUM_SERIES_GROUP	//multiple slave board
#define LTC_DELAY				2500 //500ms update delay
#define CAN1_DELAY				10
#define LED_HEARTBEAT_DELAY_MS	500  //500ms update delay
#define LTC_CMD_RDSTATA			0x0010 //Read status register group A

static const uint8_t MD_7KHZ_3KHZ = 2;
static const uint8_t CELL_CH_ALL = 0;
static const uint8_t DCP_DISABLED = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
typedef struct _GpioTimePacket {
	GPIO_TypeDef *gpio_port; //Port
	uint16_t gpio_pin;	//Pin number
	uint32_t ts_prev;	//Previous timestamp
	uint32_t ts_curr; 	//Current timestamp
} GpioTimePacket;
typedef struct _TimerPacket {
	uint32_t ts_prev;	//Previous timestamp
	uint32_t ts_curr; 	//Current timestamp
	uint32_t delay;		//Amount to delay
} TimerPacket;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void GpioTimePacket_Init(GpioTimePacket *gtp, GPIO_TypeDef *port, uint16_t pin);
void TimerPacket_Init(TimerPacket *tp, uint32_t delay);
void GpioFixedToggle(GpioTimePacket *gtp, uint16_t update_ms);
//Returns 1 at every tp->delay interval
uint8_t TimerPacket_FixedPulse(TimerPacket *tp);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	GpioTimePacket tp_led_heartbeat;
	TimerPacket timerpacket_ltc;
	TimerPacket timerpacket_can1;

	uint16_t *read_volt;
	read_volt = (uint16_t*) malloc(NUM_CELLS * sizeof(uint16_t));
	uint16_t *read_temp;
	read_temp = (uint16_t*) malloc(NUM_CELLS * sizeof(uint16_t));
	uint16_t *read_auxreg = (uint16_t*) malloc(6 * sizeof(uint16_t));
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_ADC2_Init();
	MX_CAN1_Init();
	MX_TIM7_Init();
	MX_SPI1_Init();
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 2 */
	CAN1_SettingsInit(); // Start CAN at 0x00
	//Start timer
	GpioTimePacket_Init(&tp_led_heartbeat, MCU_HEARTBEAT_LED_GPIO_Port,
	MCU_HEARTBEAT_LED_Pin);
	TimerPacket_Init(&timerpacket_ltc, LTC_DELAY);
	TimerPacket_Init(&timerpacket_can1, CAN1_DELAY);
	//Pull SPI1 nCS HIGH (deselect)
	LTC_nCS_High();
	LTC_Set_Num_Devices(NUM_DEVICES);
	LTC_Set_Num_Series_Groups(NUM_SERIES_GROUP);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	uint8_t BMS_IC[12][6] = { };
	BMS_IC[0][0] = 0x69; // Icom Start(6) + I2C_address D0 (0x90)
	BMS_IC[0][1] = 0x28; // Fcom master NACK(8)
	BMS_IC[0][2] = 0x0F; // Icom Blank (0) + eeprom address D1 (0xF8)
	BMS_IC[0][3] = 0xF9; // Fcom master NACK + Stop(9)
	BMS_IC[0][4] = 0x7F; // NO TRANSMIT
	BMS_IC[0][5] = 0xF9; // Fcom master NACK + Stop(9)

	BMS_IC[1][0] = 0x69;
	BMS_IC[1][1] = 0x28;
	BMS_IC[1][2] = 0x0F;
	BMS_IC[1][3] = 0xE9;
	BMS_IC[1][4] = 0x7F;
	BMS_IC[1][5] = 0xF9;

	BMS_IC[2][0] = 0x69;
	BMS_IC[2][1] = 0x28;
	BMS_IC[2][2] = 0x0F;
	BMS_IC[2][3] = 0xD9;
	BMS_IC[2][4] = 0x7F;
	BMS_IC[2][5] = 0xF9;

	BMS_IC[3][0] = 0x69;
	BMS_IC[3][1] = 0x28;
	BMS_IC[3][2] = 0x0F;
	BMS_IC[3][3] = 0xC9;
	BMS_IC[3][4] = 0x7F;
	BMS_IC[3][5] = 0xF9;

	BMS_IC[4][0] = 0x69;
	BMS_IC[4][1] = 0x28;
	BMS_IC[4][2] = 0x0F;
	BMS_IC[4][3] = 0xB9;
	BMS_IC[4][4] = 0x7F;
	BMS_IC[4][5] = 0xF9;

	BMS_IC[5][0] = 0x69;
	BMS_IC[5][1] = 0x28;
	BMS_IC[5][2] = 0x0F;
	BMS_IC[5][3] = 0xA9;
	BMS_IC[5][4] = 0x7F;
	BMS_IC[5][5] = 0xF9;

	BMS_IC[6][0] = 0x69;
	BMS_IC[6][1] = 0x28;
	BMS_IC[6][2] = 0x0F;
	BMS_IC[6][3] = 0x99;
	BMS_IC[6][4] = 0x7F;
	BMS_IC[6][5] = 0xF9;

	BMS_IC[7][0] = 0x69;
	BMS_IC[7][1] = 0x28;
	BMS_IC[7][2] = 0x0F;
	BMS_IC[7][3] = 0x89;
	BMS_IC[7][4] = 0x7F;
	BMS_IC[7][5] = 0xF9;

	BMS_IC[8][0] = 0x69;
	BMS_IC[8][1] = 0x08;
	BMS_IC[8][2] = 0x0F;
	BMS_IC[8][3] = 0xF9;
	BMS_IC[8][4] = 0x7F;
	BMS_IC[8][5] = 0xF9;

	BMS_IC[9][0] = 0x69;
	BMS_IC[9][1] = 0x08;
	BMS_IC[9][2] = 0x0F;
	BMS_IC[9][3] = 0xE9;
	BMS_IC[9][4] = 0x7F;
	BMS_IC[9][5] = 0xF9;

	BMS_IC[10][0] = 0x69;
	BMS_IC[10][1] = 0x08;
	BMS_IC[10][2] = 0x0F;
	BMS_IC[10][3] = 0xD9;
	BMS_IC[10][4] = 0x7F;
	BMS_IC[10][5] = 0xF9;

	BMS_IC[11][0] = 0x69;
	BMS_IC[11][1] = 0x08;
	BMS_IC[11][2] = 0x0F;
	BMS_IC[11][3] = 0xC9;
	BMS_IC[11][4] = 0x7F;
	BMS_IC[11][5] = 0xF9;
	uint8_t tempindex = 0;
	uint8_t increment = 0;
	uint16_t data = 0;
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		GpioFixedToggle(&tp_led_heartbeat, LED_HEARTBEAT_DELAY_MS);
		if (TimerPacket_FixedPulse(&timerpacket_ltc)) {
//			int packvoltage = 0;

			//starting for printing over serial
//			char packV[30];
			//end for printing over serial

			//start reading voltages
//			LTC_Wakeup_Sleep();
//			LTC_ADCV(MD_7KHZ_3KHZ, DCP_DISABLED, CELL_CH_ALL);
//			LTC_PollAdc();
//			LTC_ReadRawCellVoltages((uint16_t*) read_volt);
//			packvoltage = LTC_CalcPackVoltage((uint16_t*) read_volt);
			//stop reading voltages

			//start sending to mux to read temperatures
			LTC_Wakeup_Sleep();
			ltc6811_wrcomm(NUM_DEVICES, BMS_IC[tempindex]);
			LTC_Wakeup_Idle();
			ltc6811_stcomm();
			//end sending to mux to read temperatures



			//start for printing over serial for pack voltage
//			sprintf(packV, "Pack Voltage: %d/10000 V", packvoltage);
//			strncat(out_buf, packV, 30);
//			strncat(out_buf, char_to_str, 2);
			//end for printing over serial for pack voltage

			LTC_Wakeup_Idle();
			LTC_ADAX(MD_7KHZ_3KHZ, 0); //doing GPIO all conversion
			LTC_PollAdc();
			if (!LTC_ReadRawCellTemps((uint16_t *) read_auxreg)) // Set to read back all aux registers
			{
				data = read_auxreg[0];
				read_temp[tempindex] = data;
				//read_temp[tempindex] = (uint16_t) read_auxreg[0];

				//start for printing over serial for voltages
				print(12, (uint16_t *) read_temp);
				HAL_Delay(400);
				//end for printing over serial for voltages

				tempindex++;

				if (tempindex == 12) {
					tempindex = 0;
				}
			}
			HAL_Delay(1000);

		} //STOPSTOSPTOPSOTSTPO

//		if (TimerPacket_FixedPulse(&timerpacket_can1)) {
//
//			uint16_t CAN_ID = 0x630;
//			setCANId(CAN_ID);
//			for (int i = 0; i < NUM_CELLS; i++) {
//				if (i % 4 == 0) {
//					uint8_t temp_volt = i;
//					msg.data[0] = read_volt[temp_volt];
//					msg.data[1] = read_volt[temp_volt] >> 8;
//					temp_volt += 1;
//					msg.data[2] = read_volt[temp_volt];
//					msg.data[3] = read_volt[temp_volt] >> 8;
//					temp_volt += 1;
//					msg.data[4] = read_volt[temp_volt];
//					msg.data[5] = read_volt[temp_volt] >> 8;
//					temp_volt += 1;
//					msg.data[6] = read_volt[temp_volt];
//					msg.data[7] = read_volt[temp_volt] >> 8;
//				}
//				if (i % 4 == 0) {
//					CAN_ID = CAN_ID + 0x01;
//					setCANId(CAN_ID);
//				}
//				HAL_Delay(10);
//				CAN1_Send();
//			}
//		}
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV5;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_PLL2;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL2_ON;
	RCC_OscInitStruct.PLL2.PLL2MUL = RCC_PLL2_MUL8;
	RCC_OscInitStruct.PLL2.HSEPrediv2Value = RCC_HSE_PREDIV2_DIV5;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC | RCC_PERIPHCLK_USB;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV3;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}

	/** Configure the Systick interrupt time
	 */
	__HAL_RCC_PLLI2S_ENABLE();
}

/* USER CODE BEGIN 4 */
//Initialize struct values
//Will initialize GPIO to LOW!
void GpioTimePacket_Init(GpioTimePacket *gtp, GPIO_TypeDef *port, uint16_t pin) {
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET); //Set GPIO LOW
	gtp->gpio_port = port;
	gtp->gpio_pin = pin;
	gtp->ts_prev = 0; //Init to 0
	gtp->ts_curr = 0; //Init to 0
}
//update_ms = update after X ms
void GpioFixedToggle(GpioTimePacket *gtp, uint16_t update_ms) {
	gtp->ts_curr = HAL_GetTick(); //Record current timestamp
	if (gtp->ts_curr - gtp->ts_prev > update_ms) {
		HAL_GPIO_TogglePin(gtp->gpio_port, gtp->gpio_pin); // Toggle GPIO
		gtp->ts_prev = gtp->ts_curr;
	}
}
//Initialize struct values
//Will initialize GPIO to LOW!
void TimerPacket_Init(TimerPacket *tp, uint32_t delay) {
	tp->ts_prev = 0;		//Init to 0
	tp->ts_curr = 0; 		//Init to 0
	tp->delay = delay;	//Init to user value
}
//update_ms = update after X ms
uint8_t TimerPacket_FixedPulse(TimerPacket *tp) {
	tp->ts_curr = HAL_GetTick(); //Record current timestamp
	if (tp->ts_curr - tp->ts_prev > tp->delay) {
		tp->ts_prev = tp->ts_curr; //Update prev timestamp to current
		return 1; //Enact event (time interval is a go)
	}
	return 0; //Do not enact event
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
