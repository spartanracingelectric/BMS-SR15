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
#include "module.h"
#include "safety.h"
#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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
uint8_t config[][6] = {{ 0xF8, 0x00, 0x00, 0x00, 0x01, 0x00 }};
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	GpioTimePacket tp_led_heartbeat;
	TimerPacket timerpacket_ltc_volt;
	TimerPacket timerpacket_ltc_temp;
	TimerPacket timerpacket_can;
	TimerPacket timerpacket_safety;

	struct batteryModuleVoltage modVoltage = { .cell_volt = (uint16_t*) malloc(
	NUM_CELLS * sizeof(uint16_t)), .cell_temp = (uint16_t*) malloc(
	NUM_THERM_TOTAL * sizeof(uint16_t)), .read_auxreg = (uint16_t*) malloc(
	NUM_AUXES * sizeof(uint16_t)) };

	struct CANMessage msg;
	uint8_t safetyFaults = 0;
	uint8_t safetyWarnings = 0;

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
	MX_TIM7_Init();
	MX_SPI1_Init();
	MX_USB_DEVICE_Init();
	MX_CAN2_Init();
	/* USER CODE BEGIN 2 */
	CAN_SettingsInit(&msg); // Start CAN at 0x00
	//Start timer
	GpioTimePacket_Init(&tp_led_heartbeat, MCU_HEARTBEAT_LED_GPIO_Port,
	MCU_HEARTBEAT_LED_Pin);
	TimerPacket_Init(&timerpacket_ltc_volt, LTC_VOlT_DELAY);
	TimerPacket_Init(&timerpacket_ltc_temp, LTC_TEMP_DELAY);
	TimerPacket_Init(&timerpacket_can, CAN_DELAY);
	TimerPacket_Init(&timerpacket_safety, SAFETY_DELAY);
	//Pull SPI1 nCS HIGH (deselect)
	LTC_nCS_High();
	set_num_devices(NUM_DEVICES);
	set_series_groups(NUM_CELL_SERIES_GROUP);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	uint8_t tempindex = 0;
	uint8_t indexpause = 8;
	uint8_t loop_count = 3;

	// Config Balancing controls: 
	// 4'b1111 for no balance 
	// 4'b0000 for balance 

	wakeup_sleep();
	ltc6811_wrpwm(NUM_DEVICES, 0x00);
	// TODO test discharge by turning on DCC bits. 


	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		GpioFixedToggle(&tp_led_heartbeat, LED_HEARTBEAT_DELAY_MS);
		if (TimerPacket_FixedPulse(&timerpacket_ltc_volt)) {
			wakeup_sleep();
			readVolt(modVoltage.cell_volt);
			wakeup_idle(); // Sets up for balance.
			ltc6811_wrcfg(NUM_DEVICES, config);
			print(NUM_CELLS, (uint16_t*) modVoltage.cell_volt);
		}

		if (TimerPacket_FixedPulse(&timerpacket_ltc_temp)) {
			//start sending to mux to read temperatures
			wakeup_sleep();
			for (uint8_t i = tempindex; i < indexpause; i++) {
				readTemp(i, modVoltage.cell_temp, modVoltage.read_auxreg);
				HAL_Delay(100);
			}
			if (indexpause == 8) {
				tempindex = 8;
				indexpause = 12;
			} else if (indexpause == 12) {
				indexpause = 8;
				tempindex = 0;
			}
			HAL_Delay(2300);
			//print(NUM_THERM_TOTAL, (uint16_t*) modVoltage.cell_temp);
		}

		if (loop_count == 0) {
			if (TimerPacket_FixedPulse(&timerpacket_safety)) {
				cellSummary(&modVoltage);
				fault_and_warning(&modVoltage, &safetyFaults, &safetyWarnings);
				if (safetyFaults != 0) {
					HAL_GPIO_WritePin(Fault_GPIO_Port, Fault_Pin, GPIO_PIN_SET);
				}

			}
		} else {
			loop_count--;
		}

		if (TimerPacket_FixedPulse(&timerpacket_can)) {
			CAN_Send_Safety_Checker(&msg,&safetyFaults, &safetyWarnings);
			CAN_Send_Cell_Summary(&msg, &modVoltage);
			CAN_Send_Voltage(&msg, modVoltage.cell_volt);
			CAN_Send_Temperature(&msg, modVoltage.cell_temp);

		}

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
