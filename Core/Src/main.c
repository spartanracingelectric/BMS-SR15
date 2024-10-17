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
#include "main.h"
#include "adc.h"
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

#include "string.h"
#include "6811.h"
#include "print.h"
#include "module.h"
#include "safety.h"
#include "usbd_cdc_if.h"
#include "balance.h"


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

void SystemClock_Config(void);

void GpioTimePacket_Init(GpioTimePacket *gtp, GPIO_TypeDef *port, uint16_t pin);
void TimerPacket_Init(TimerPacket *tp, uint32_t delay);
void GpioFixedToggle(GpioTimePacket *gtp, uint16_t update_ms);
uint8_t TimerPacket_FixedPulse(TimerPacket *tp);

void InitPeripherals(
    struct CANMessage *msg
    GpioTimePacket *tp_led_heartbeat,
    TimerPacket *timerpacket_ltc
);
void ReadCellThermTemps(struct batteryModule *modPackInfo);
void SendCAN(
    struct CANMessage *msg,
    struct batteryModule *modPackInfo,
    uint8_t safetyFaults,
    uint8_t safetyWarnings,
    uint8_t safetyStates
);
void PassiveBalance(struct batteryModule *modPackInfo, uint8_t *safetyFaults);
void InitFaultSignal(void);

static uint8_t BMS_MUX_PAUSE[2][6] = { { 0x69, 0x28, 0x0F, 0x09, 0x7F, 0xF9 }, {
		0x69, 0x08, 0x0F, 0x09, 0x7F, 0xF9 } };

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	GpioTimePacket tp_led_heartbeat;
	TimerPacket timerpacket_ltc;

	struct batteryModule modPackInfo;
	struct CANMessage msg;
	uint8_t safetyFaults = 0;
	uint8_t safetyWarnings = 0;
	uint8_t safetyStates = 0;

	uint8_t tempindex = 0;
	uint8_t indexpause = 8;
	uint8_t low_volt_hysteresis = 0;
	uint8_t high_volt_hysteresis = 0;
	uint8_t cell_imbalance_hysteresis = 0;

    InitPeripherals(&msg);
    InitFaultSignal();

	Read_Volt(modPackInfo.cell_volt);
    ReadCellThermTemps(&modPackInfo);

	while (1) {
		GpioFixedToggle(&tp_led_heartbeat, LED_HEARTBEAT_DELAY_MS);
		if (TimerPacket_FixedPulse(&timerpacket_ltc)) {
            SendCAN(
                &modPackInfo, &msg, &safetyFaults, &safetyWarnings, &safetyStates
            );

			Read_Volt(modPackInfo.cell_volt);
			//print(NUM_CELLS, (uint16_t*) modPackInfo.cell_volt);

			//reading cell temperatures
			Wakeup_Sleep();
			for (uint8_t i = tempindex; i < indexpause; i++) {
				Read_Temp(i, modPackInfo.cell_temp, modPackInfo.read_auxreg);
			}
			if (indexpause == 8) {
				LTC_WRCOMM(NUM_DEVICES, BMS_MUX_PAUSE[0]);
				LTC_STCOMM(2);
				tempindex = 8;
				indexpause = NUM_THERM_PER_MOD;
			} else if (indexpause == NUM_THERM_PER_MOD) {
				LTC_WRCOMM(NUM_DEVICES, BMS_MUX_PAUSE[1]);
				LTC_STCOMM(2);
				indexpause = 8;
				tempindex = 0;
			}
			//print(NUM_THERM_TOTAL, (uint16_t*) modPackInfo.cell_temp);

			Cell_Summary(&modPackInfo);

			//checking for faults
			Fault_Warning_State(
                &modPackInfo, &safetyFaults,
                &safetyWarnings, &safetyStates, &low_volt_hysteresis,
                &high_volt_hysteresis, &cell_imbalance_hysteresis
            );

			if (safetyFaults != 0) {
				HAL_GPIO_WritePin(Fault_GPIO_Port, Fault_Pin, GPIO_PIN_SET);
			}

            PassiveBalance(&modPackInfo, &safetyFaults);
		}
	}
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

/*
 * Initialize fault signal
 */
void InitFaultSignal(void) {
	// Sends and resets a fault signal
	HAL_GPIO_WritePin(Fault_GPIO_Port, Fault_Pin, GPIO_PIN_SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(Fault_GPIO_Port, Fault_Pin, GPIO_PIN_RESET);
}

/*
 * Read cell temperatures from thermistors
 */
void ReadCellThermTemps(struct batteryModule *modPackInfo) {
	Wakeup_Sleep();
	for (uint8_t i = 0; i < 8; i++) {
		Read_Temp(i, modPackInfo->cell_temp, modPackInfo->read_auxreg);
	}
	LTC_WRCOMM(NUM_DEVICES, BMS_MUX_PAUSE[0]);
	LTC_STCOMM(2);

	Wakeup_Sleep();
	for (uint8_t i = 8; i < NUM_THERM_PER_MOD; i++) {
		Read_Temp(i, modPackInfo->cell_temp, modPackInfo->read_auxreg);
	}
	LTC_WRCOMM(NUM_DEVICES, BMS_MUX_PAUSE[1]);
	LTC_STCOMM(2);
}

/*
 * Call passive balancing unless a fault has occurred
 */
void PassiveBalance(struct batteryModule *modPackInfo, uint8_t *safetyFaults) {
    if (
        *safetyFaults == 0 && BALANCE &&
        (modPackInfo->cell_volt_highest - modPackInfo->cell_volt_lowest) > 50
    ) {
        Start_Balance((uint16_t*) modPackInfo->cell_volt,
        NUM_DEVICES, modPackInfo->cell_volt_lowest);
    } else if (BALANCE) {
        End_Balance(safetyFaults);
    }
}


/**
 * Send cell information to CAN
 */
void SendCAN(
    struct CANMessage *msg,
    struct batteryModule *modPackInfo,
    uint8_t *safetyFaults,
    uint8_t *safetyWarnings,
    uint8_t *safetyStates
) {
    CAN_Send_Safety_Checker(
        msg,
        modPackInfo,
        safetyFaults,
        safetyWarnings,
        safetyStates
    );
    CAN_Send_Cell_Summary(msg, modPackInfo);
    CAN_Send_Voltage(msg, modPackInfo->cell_volt);
    CAN_Send_Temperature(msg, modPackInfo->cell_temp);
}

/**
 * Initialize configured Peripherals
 */
void InitPeripherals(
    struct CANMessage *msg,
    GpioTimePacket *tp_led_heartbeat,
    TimerPacket *timerpacket_ltc
) {
	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_ADC2_Init();
	MX_TIM7_Init();
	MX_SPI1_Init();
	MX_USB_DEVICE_Init();
	MX_CAN2_Init();

    // Start CAN at 0x00
	CAN_SettingsInit(msg);
                           
	//Start timer
	GpioTimePacket_Init(tp_led_heartbeat, MCU_HEARTBEAT_LED_GPIO_Port,
	MCU_HEARTBEAT_LED_Pin);
	TimerPacket_Init(timerpacket_ltc, LTC_DELAY);

	//Pull SPI1 nCS HIGH (deselect)
	LTC_nCS_High();
}

/**
 * Intialize GpioTimePacket struct values
 * NOTE: Will initialize GPIO to LOW!
 */
void GpioTimePacket_Init(GpioTimePacket *gtp, GPIO_TypeDef *port, uint16_t pin) {
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET); //Set GPIO LOW
	gtp->gpio_port = port;
	gtp->gpio_pin = pin;
	gtp->ts_prev = 0; //Init to 0
	gtp->ts_curr = 0; //Init to 0
}

void GpioFixedToggle(GpioTimePacket *gtp, uint16_t update_ms) {
    // update_ms = update after X ms
	gtp->ts_curr = HAL_GetTick(); //Record current timestamp
	if (gtp->ts_curr - gtp->ts_prev > update_ms) {
		HAL_GPIO_TogglePin(gtp->gpio_port, gtp->gpio_pin); // Toggle GPIO
		gtp->ts_prev = gtp->ts_curr;
	}
}

/**
 * Intialize TimerPacket struct values
 * NOTE: Will initialize GPIO to LOW!
 */
void TimerPacket_Init(TimerPacket *tp, uint32_t delay) {
	tp->ts_prev = 0;		//Init to 0
	tp->ts_curr = 0; 		//Init to 0
	tp->delay = delay;	//Init to user value
}

uint8_t TimerPacket_FixedPulse(TimerPacket *tp) {
    // update_ms = update after X ms
    // Returns 1 at every tp->delay interval
	tp->ts_curr = HAL_GetTick(); //Record current timestamp
	if (tp->ts_curr - tp->ts_prev > tp->delay) {
		tp->ts_prev = tp->ts_curr; //Update prev timestamp to current
		return 1; //Enact event (time interval is a go)
	}
	return 0; //Do not enact event
}

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
