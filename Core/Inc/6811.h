/*
 * 6811.h
 *
 *  Created on: Nov 3, 2023
 *      Author: karth
 */

#include <stdint.h>
#include "main.h"
#include "spi.h"
#include "string.h"

#ifndef INC_6811_H_
#define INC_6811_H_



#endif /* INC_6811_H_ */

typedef enum
{
  LTC_SPI_OK       		= 0x00U, //0b00000000
  LTC_SPI_TX_ERROR  	= 0x02U, //0b00000010
  LTC_SPI_TX_BUSY   	= 0x04U, //0b00000100
  LTC_SPI_TX_TIMEOUT  	= 0x08U, //0b00001000
  LTC_SPI_RX_ERROR  	= 0x20U, //0b00100000
  LTC_SPI_RX_BUSY   	= 0x40U, //0b01000000
  LTC_SPI_RX_TIMEOUT  	= 0x80U	 //0b10000000
} LTC_SPI_StatusTypeDef;

uint16_t LTC_PEC15_Calc(uint8_t len, //Number of bytes that will be used to calculate a PEC
                    uint8_t *data //Array of data that will be used to calculate a PEC
                   );

/* Set number of LTC6813/slave devices */
void LTC_Set_Num_Devices(uint8_t num);

/* Get number of LTC6813/slave devices */
uint8_t LTC_Get_Num_Devices(void);

/* Set number of series groups per LTC6813/slave */
void LTC_Set_Num_Series_Groups(uint8_t num);

/* Get number of series groups per LTC6813/slave */
uint8_t LTC_Get_Num_Series_Groups(void);

LTC_SPI_StatusTypeDef LTC_Wakeup_Idle(void);

LTC_SPI_StatusTypeDef LTC_ReadRawCellVoltages(uint16_t *read_voltages);


