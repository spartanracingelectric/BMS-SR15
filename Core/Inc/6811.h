#include <stdint.h>
#include <stdlib.h>
#include "main.h"
#include "spi.h"
#include "string.h"

#ifndef INC_6811_H_
#define INC_6811_H_

#endif /* INC_6811_H_ */

typedef enum {
	LTC_SPI_OK = 0x00U, //0b00000000
	LTC_SPI_TX_ERROR = 0x02U, //0b00000010
	LTC_SPI_TX_BUSY = 0x04U, //0b00000100
	LTC_SPI_TX_TIMEOUT = 0x08U, //0b00001000
	LTC_SPI_RX_ERROR = 0x20U, //0b00100000
	LTC_SPI_RX_BUSY = 0x40U, //0b01000000
	LTC_SPI_RX_TIMEOUT = 0x80U	 //0b10000000
} LTC_SPI_StatusTypeDef;

void Wakeup_Idle(void);

void Wakeup_Sleep(void);

LTC_SPI_StatusTypeDef Read_Cell_Volt(uint16_t *read_voltages);

/* write to PWM register to control balancing functionality */
void LTC6811_WRPWM(uint8_t total_ic, uint8_t pwm);

void LTC6811_WRCFG(uint8_t total_ic, //The number of ICs being written to
		uint8_t config[][6] //A two dimensional array of the configuration data that will be written
		);

void LTC_WRCOMM(uint8_t total_ic, //The number of ICs being written to
		uint8_t comm[6] //A two dimensional array of the comm data that will be written
		);

void LTC_STCOMM(uint8_t len);

LTC_SPI_StatusTypeDef Read_Cell_Temps(uint16_t *read_auxiliary);

void LTC_ADCV(uint8_t MD, //ADC Mode
		uint8_t DCP, //Discharge Permit
		uint8_t CH //Cell Channels to be measured
		);

void LTC_ADAX(uint8_t MD, //ADC Mode
		uint8_t CHG //GPIO Channels to be measured)
		);

int32_t LTC_POLLADC();

int Calc_Pack_Voltage(uint16_t *read_voltages);

uint16_t LTC_Pec15_Calc(uint8_t len, //Number of bytes that will be used to calculate a PEC
		uint8_t *data //Array of data that will be used to calculate a PEC
		);

