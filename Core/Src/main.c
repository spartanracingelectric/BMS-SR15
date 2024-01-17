/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "6811.h"

//#include "ltc6811.h"

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
	GPIO_TypeDef 	*gpio_port; //Port
	uint16_t		gpio_pin;	//Pin number
	uint32_t 		ts_prev;	//Previous timestamp
	uint32_t 		ts_curr; 	//Current timestamp
} GpioTimePacket;

typedef struct _TimerPacket {
	uint32_t 		ts_prev;	//Previous timestamp
	uint32_t 		ts_curr; 	//Current timestamp
	uint32_t		delay;		//Amount to delay
} TimerPacket;

#define NUM_CELLS_PER_PACKET 4 //number of cells that an be read at once
#define offsetCellMACRO (NUM_CELLS_PER_PACKET * currentModule) //use to interate through voltage array

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
int main(void)
{
  /* USER CODE BEGIN 1 */
//	GpioTimePacket tp_led_heartbeat;
	TimerPacket timerpacket_ltc;
	uint16_t read_volt[NUM_CELLS]; //2 bytes per series * 12 series
	uint16_t read_temp[12];
	float actual_temp[12]; //only 10 actual temps, but 12 for rn cuz not enough time to mess with the loop

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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  CAN1_SettingsInit(); // Start CAN at 0x00

  //Pull SPI1 nCS HIGH (deselect)
  LTC_nCS_High();
  LTC_Set_Num_Devices(NUM_DEVICES);
  LTC_Set_Num_Series_Groups(NUM_SERIES_GROUP);

  /* This enum holds all states of the FSM to output can messages for battery modules
   * There are 8 modules, each module will have 3 parts A, B, C
   */


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		GpioFixedToggle(&tp_led_heartbeat, LED_HEARTBEAT_DELAY_MS);


	  // Iterate through each module



//		if (TimerPacket_FixedPulse(&timerpacket_ltc)) {
			char packV[30];
			char buf[20];
			char out_buf[2048] = "";
			char char_to_str[2];
			int packvoltage = 0;



			LTC_ADCV(MD_7KHZ_3KHZ,DCP_DISABLED,CELL_CH_ALL);
			LTC_PollAdc();
			LTC_ReadRawCellVoltages((uint16_t *)read_volt);
			packvoltage = LTC_CalcPackVoltage((uint16_t *) read_volt);

							      // Package High bits and low bits of voltage readings into buffer
			uint16_t CAN_ID = 0x630;
			setCANId(CAN_ID);

//			uint16_t read_volt[96] = {0};

//
//						uint16_t read_volt[96] = {
//									    0x0000, 0x002B, 0x0057, 0x0083,
//									    0x00AF, 0x00DA, 0x0106, 0x0132,
//									    0x015E, 0x0189, 0x01B5, 0x01E1,
//									    0x020D, 0x0238, 0x0264, 0x0290,
//									    0x02BC, 0x02E7, 0x0313, 0x033F,
//									    0x036B, 0x0396, 0x03C2, 0x03EE,
//									    0x041A, 0x0446, 0x0471, 0x049D,
//									    0x04C9, 0x04F5, 0x0520, 0x054C,
//									    0x0578, 0x05A4, 0x05CF, 0x05FB,
//									    0x0627, 0x0653, 0x067F, 0x06AA,
//									    0x06D6, 0x0702, 0x072E, 0x0759,
//									    0x0785, 0x07B1, 0x07DD, 0x0808,
//									    0x0834, 0x0860, 0x088C, 0x08B7,
//									    0x08E3, 0x090F, 0x093B, 0x0966,
//									    0x0992, 0x09BE, 0x09EA, 0x0A15,
//									    0x0A41, 0x0A6D, 0x0A99, 0x0AC4,
//									    0x0AF0, 0x0B1C, 0x0B48, 0x0B73,
//									    0x0B9F, 0x0BCB, 0x0BF7, 0x0C22,
//									    0x0C4E, 0x0C7A, 0x0CA6, 0x0CD1,
//									    0x0CFD, 0x0D29, 0x0D55, 0x0D80,
//									    0x0DAC, 0x0DD8, 0x0E04, 0x0E2F,
//									    0x0E5B, 0x0E87, 0x0EB3, 0x0EDE,
//									    0x0F0A, 0x0F36, 0x0F62, 0x0F8D,
//									    0x0FB9, 0x0FE5, 0x1011, 0x103C
//						};

			uint8_t index = 0;

			for(uint8_t currentModule = 0; currentModule < NUM_MODULES; currentModule++){

//				// Cell 1
//				msg.data[0] = (uint8_t)read_volt[i];
//				msg.data[1] = read_volt[i] >> 8;
//
//				// Cell 2
//				msg.data[2] = (uint8_t)read_volt[i + 1];
//				msg.data[3] = read_volt[i + 1] >> 8;
//
//				// Cell 3
//				msg.data[4] = (uint8_t)read_volt[i + 2];
//				msg.data[5] = read_volt[i + 2] >> 8;
//
//				// Cell 4
//				msg.data[6] = (uint8_t)read_volt[i + 3];
//				msg.data[7] = read_volt[i + 3] >> 8;





			      // Cell 0
				msg.data[0] = (uint8_t)read_volt[0 + offsetCellMACRO];
				msg.data[1] = read_volt[0 + offsetCellMACRO] >> 8;

			      // Cell 1
				msg.data[2] = (uint8_t)read_volt[1 + offsetCellMACRO];
				msg.data[3] = read_volt[1 + offsetCellMACRO] >> 8;

			      // Cell 2
				msg.data[4] = (uint8_t)read_volt[2 + offsetCellMACRO];
				msg.data[5] = read_volt[2 + offsetCellMACRO] >> 8;

			      // Cell 3
				msg.data[6] = (uint8_t)read_volt[3 + offsetCellMACRO];
				msg.data[7] = read_volt[3 + offsetCellMACRO] >> 8;












							      // Send out the packet
				CAN1_Send();

//				if(index % 25 != 0 ){
//					CAN_ID++;
//				} else {
//					CAN_ID = 0x630;
//				}
//					index++;
//					setCANId(CAN_ID);
				    HAL_Delay(1000);

			}
							   // This delays the execution for 1000 milliseconds (1 second)



//			sprintf(packV, "Pack Voltage: %d/10000 V", packvoltage);
//			strncat(out_buf, packV, 30);
//			strncat(out_buf, char_to_str, 2);
//
//
			char_to_str[0] = '\n';
			char_to_str[1] = '\0';
//
//
//			for (uint8_t i = 0; i < NUM_CELLS; i++) {
//				sprintf(buf, "C%u:%u/10000 V", i+1, read_volt[i]);
//				strncat(out_buf, buf, 20);
//				strncat(out_buf, char_to_str, 2);
//			}
//			strncat(out_buf, char_to_str, 2);
//
//			USB_Transmit(out_buf, strlen(out_buf));

			char buf2[20];
			char out_buf2[2048] = "";

			LTC_Wakeup_Idle();
			LTC_ADAX(MD_7KHZ_3KHZ, AUX_CH_ALL);
			LTC_PollAdc();
			LTC_ReadRawCellTemps((uint16_t *) read_temp); // Set to read back all aux registers
			for (uint8_t i = 0; i < 12; i++) {
				getActualTemps(actual_temp, read_temp);
				//sprintf(buf2, "Vref:%u", read_temp[i]);
//				sprintf(buf2, "temp: %0.2f", actual_temp[i]);
//				strncat(out_buf2, buf2, 20);
//				strncat(out_buf2, char_to_str, 2);
			}
			strncat(out_buf2, char_to_str, 2);



//			USB_Transmit(out_buf2, strlen(out_buf2));
		}





  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_ENABLE();
}

/* USER CODE BEGIN 4 */

//Initialize struct values
//Will initialize GPIO to LOW!
void GpioTimePacket_Init(GpioTimePacket *gtp, GPIO_TypeDef *port, uint16_t pin)
{
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET); //Set GPIO LOW
	gtp->gpio_port	= port;
	gtp->gpio_pin	= pin;
	gtp->ts_prev 	= 0; //Init to 0
	gtp->ts_curr 	= 0; //Init to 0
}

//update_ms = update after X ms
void GpioFixedToggle(GpioTimePacket *gtp, uint16_t update_ms)
{
	gtp->ts_curr = HAL_GetTick(); //Record current timestamp

	if (gtp->ts_curr - gtp->ts_prev > update_ms) {
		HAL_GPIO_TogglePin(gtp->gpio_port, gtp->gpio_pin); // Toggle GPIO
		gtp->ts_prev = gtp->ts_curr;
	}
}

//Initialize struct values
//Will initialize GPIO to LOW!
void TimerPacket_Init(TimerPacket *tp, uint32_t delay)
{
	tp->ts_prev 	= 0;		//Init to 0
	tp->ts_curr 	= 0; 		//Init to 0
	tp->delay		= delay;	//Init to user value
}

//update_ms = update after X ms
uint8_t TimerPacket_FixedPulse(TimerPacket *tp)
{
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
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
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
