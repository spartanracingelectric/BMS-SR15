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

void wakeup_idle(void);

void wakeup_sleep(void);

LTC_SPI_StatusTypeDef read_cell_volt(uint16_t *read_voltages);

/* write to PWM register to control balancing functionality */
void ltc6811_wrpwm(uint8_t total_ic, uint8_t pwm);

void ltc6811_wrcfg(uint8_t total_ic, //The number of ICs being written to
		uint8_t config[][6] //A two dimensional array of the configuration data that will be written
		);

void ltc_wrcomm(uint8_t total_ic, //The number of ICs being written to
		uint8_t comm[6] //A two dimensional array of the comm data that will be written
		);

void ltc_stcomm(uint8_t len);

LTC_SPI_StatusTypeDef read_cell_temps(uint16_t *read_auxiliary);

void ltc_adcv(uint8_t MD, //ADC Mode
		uint8_t DCP, //Discharge Permit
		uint8_t CH //Cell Channels to be measured
		);

void ltc_adax(uint8_t MD, //ADC Mode
		uint8_t CHG //GPIO Channels to be measured)
		);

int32_t ltc_polladc();

int calc_pack_voltage(uint16_t *read_voltages);

uint16_t ltc_pec15_calc(uint8_t len, //Number of bytes that will be used to calculate a PEC
		uint8_t *data //Array of data that will be used to calculate a PEC
		);

