/*
 * 6811.c
 *
 *  Created on: Nov 3, 2023
 *      Author: karth
 */
#include "6811.h"

static const uint16_t LTC_CMD_RDCVA = 0x0004;
static const uint16_t LTC_CMD_RDCVB = 0x0006;
static const uint16_t LTC_CMD_RDCVC = 0x0008;
static const uint16_t LTC_CMD_RDCVD = 0x000A;

static const uint16_t LTC_CMD_RDCV[4] = {
										LTC_CMD_RDCVA,
										LTC_CMD_RDCVB,
										LTC_CMD_RDCVC,
										LTC_CMD_RDCVD
										};

static const uint8_t LTC_SPI_TX_BIT_OFFSET = 0; //Num bits to shift RX status code
static const uint8_t LTC_SPI_RX_BIT_OFFSET = 4; //Num bits to shift RX status code
static const uint8_t REG_LEN = 8; // number of bytes in the register + 2 bytes for the PEC
static const uint8_t LTC_SERIES_GROUPS_PER_RDCV = 3; //Number of cell voltage groups per 8 byte register
static uint8_t num_devices; //Keep visibility within this file
static uint8_t num_series_groups; //Number of series groups


static const unsigned int crc15Table[256] = {0x0,0xc599, 0xceab, 0xb32, 0xd8cf, 0x1d56, 0x1664, 0xd3fd, 0xf407, 0x319e, 0x3aac,  //!<precomputed CRC15 Table
    0xff35, 0x2cc8, 0xe951, 0xe263, 0x27fa, 0xad97, 0x680e, 0x633c, 0xa6a5, 0x7558, 0xb0c1,
    0xbbf3, 0x7e6a, 0x5990, 0x9c09, 0x973b, 0x52a2, 0x815f, 0x44c6, 0x4ff4, 0x8a6d, 0x5b2e,
    0x9eb7, 0x9585, 0x501c, 0x83e1, 0x4678, 0x4d4a, 0x88d3, 0xaf29, 0x6ab0, 0x6182, 0xa41b,
    0x77e6, 0xb27f, 0xb94d, 0x7cd4, 0xf6b9, 0x3320, 0x3812, 0xfd8b, 0x2e76, 0xebef, 0xe0dd,
    0x2544, 0x2be, 0xc727, 0xcc15, 0x98c, 0xda71, 0x1fe8, 0x14da, 0xd143, 0xf3c5, 0x365c,
    0x3d6e, 0xf8f7,0x2b0a, 0xee93, 0xe5a1, 0x2038, 0x7c2, 0xc25b, 0xc969, 0xcf0, 0xdf0d,
    0x1a94, 0x11a6, 0xd43f, 0x5e52, 0x9bcb, 0x90f9, 0x5560, 0x869d, 0x4304, 0x4836, 0x8daf,
    0xaa55, 0x6fcc, 0x64fe, 0xa167, 0x729a, 0xb703, 0xbc31, 0x79a8, 0xa8eb, 0x6d72, 0x6640,
    0xa3d9, 0x7024, 0xb5bd, 0xbe8f, 0x7b16, 0x5cec, 0x9975, 0x9247, 0x57de, 0x8423, 0x41ba,
    0x4a88, 0x8f11, 0x57c, 0xc0e5, 0xcbd7, 0xe4e, 0xddb3, 0x182a, 0x1318, 0xd681, 0xf17b,
    0x34e2, 0x3fd0, 0xfa49, 0x29b4, 0xec2d, 0xe71f, 0x2286, 0xa213, 0x678a, 0x6cb8, 0xa921,
    0x7adc, 0xbf45, 0xb477, 0x71ee, 0x5614, 0x938d, 0x98bf, 0x5d26, 0x8edb, 0x4b42, 0x4070,
    0x85e9, 0xf84, 0xca1d, 0xc12f, 0x4b6, 0xd74b, 0x12d2, 0x19e0, 0xdc79, 0xfb83, 0x3e1a, 0x3528,
    0xf0b1, 0x234c, 0xe6d5, 0xede7, 0x287e, 0xf93d, 0x3ca4, 0x3796, 0xf20f, 0x21f2, 0xe46b, 0xef59,
    0x2ac0, 0xd3a, 0xc8a3, 0xc391, 0x608, 0xd5f5, 0x106c, 0x1b5e, 0xdec7, 0x54aa, 0x9133, 0x9a01,
    0x5f98, 0x8c65, 0x49fc, 0x42ce, 0x8757, 0xa0ad, 0x6534, 0x6e06, 0xab9f, 0x7862, 0xbdfb, 0xb6c9,
    0x7350, 0x51d6, 0x944f, 0x9f7d, 0x5ae4, 0x8919, 0x4c80, 0x47b2, 0x822b, 0xa5d1, 0x6048, 0x6b7a,
    0xaee3, 0x7d1e, 0xb887, 0xb3b5, 0x762c, 0xfc41, 0x39d8, 0x32ea, 0xf773, 0x248e, 0xe117, 0xea25,
    0x2fbc, 0x846, 0xcddf, 0xc6ed, 0x374, 0xd089, 0x1510, 0x1e22, 0xdbbb, 0xaf8, 0xcf61, 0xc453,
    0x1ca, 0xd237, 0x17ae, 0x1c9c, 0xd905, 0xfeff, 0x3b66, 0x3054, 0xf5cd, 0x2630, 0xe3a9, 0xe89b,
    0x2d02, 0xa76f, 0x62f6, 0x69c4, 0xac5d, 0x7fa0, 0xba39, 0xb10b, 0x7492, 0x5368, 0x96f1, 0x9dc3,
    0x585a, 0x8ba7, 0x4e3e, 0x450c, 0x8095
                                            };

uint16_t LTC_PEC15_Calc(uint8_t len, //Number of bytes that will be used to calculate a PEC
                    uint8_t *data //Array of data that will be used to calculate a PEC
                   )
{
	uint16_t remainder, addr;
	remainder = 16;	//Initialize the PEC to 0x10000

	for (uint8_t i = 0; i<len; i++) // loops for each byte in data array
	{
		addr = ((remainder>>7)^data[i])&0xff;//calculate PEC table address
		remainder = (remainder<<8)^crc15Table[addr];
	}

	return(remainder*2);//The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2
}

/* Set number of LTC6813/slave devices */
void LTC_Set_Num_Devices(uint8_t num) {
	if (num) num_devices = num; //Non-zero
}

/* Get number of LTC6813/slave devices */
uint8_t LTC_Get_Num_Devices(void) {
	return num_devices;
}

/* Set number of series groups per LTC6813/slave */
void LTC_Set_Num_Series_Groups(uint8_t num) {
	if (num && (num <= 18)) num_series_groups = num; //Non-zero and 18 or less
}

/* Get number of series groups per LTC6813/slave */
uint8_t LTC_Get_Num_Series_Groups(void) {
	return num_series_groups;
}

/* Wake LTC up from IDLE state into READY state */
LTC_SPI_StatusTypeDef LTC_Wakeup_Idle(void) {
	LTC_SPI_StatusTypeDef ret = LTC_SPI_OK;
	LTC_SPI_StatusTypeDef hal_ret;
	uint8_t hex_ff = 0xFF;

	LTC_nCS_Low(); //Pull CS low

	for (int i = 0; i < num_devices; i++){
		hal_ret = HAL_SPI_Transmit(&hspi1, &hex_ff, 1, 100); //Send byte 0xFF to wake LTC up
		if (hal_ret) { //Non-zero means error
			//Shift 1 by returned HAL_StatusTypeDef value to get LTC_SPI_StatusTypeDef equivalent
			ret |= (1 << (hal_ret+LTC_SPI_TX_BIT_OFFSET)); //TX error
		}
	}

	LTC_nCS_High(); //Pull CS high

	return ret;
}


//wake up sleep
void LTC_Wakeup_Sleep(void) {
	LTC_nCS_Low();
	for(int i = 0; i < num_devices; i++){
		HAL_Delay(500);
	}
	LTC_nCS_High();
}


/* Read and store raw cell voltages at uint8_t 2d pointer */
LTC_SPI_StatusTypeDef LTC_ReadRawCellVoltages(uint16_t *read_voltages) {
  LTC_SPI_StatusTypeDef ret = LTC_SPI_OK;
  LTC_SPI_StatusTypeDef hal_ret;
  const uint8_t ARR_SIZE_REG = LTC_Get_Num_Devices() * REG_LEN;
  uint8_t read_voltages_reg[ARR_SIZE_REG]; // Increased in size to handle multiple devices

  for (uint8_t i = 0; i < (LTC_Get_Num_Series_Groups() / LTC_SERIES_GROUPS_PER_RDCV); i++) {
    uint8_t cmd[4];
    uint16_t cmd_pec;

    cmd[0] = (0xFF & (LTC_CMD_RDCV[i] >> 8)); //RDCV Register
    cmd[1] = (0xFF & (LTC_CMD_RDCV[i])); //RDCV Register
    cmd_pec = LTC_PEC15_Calc(2, cmd);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);

    //ret |= LTC_Wakeup_Idle(); //Wake LTC up

    LTC_nCS_Low(); //Pull CS low

    hal_ret = HAL_SPI_Transmit(&hspi1, (uint8_t *)cmd, 4, 100);
    if (hal_ret) { //Non-zero means error
      ret |= (1 << (hal_ret + LTC_SPI_TX_BIT_OFFSET)); //TX error
    }

    hal_ret = HAL_SPI_Receive(&hspi1, (uint8_t *)read_voltages_reg, ARR_SIZE_REG, 100);
    if (hal_ret) { //Non-zero means error
      ret |= (1 << (hal_ret + LTC_SPI_RX_BIT_OFFSET)); //RX error
    }

    // Process the received data
    for (uint8_t dev_idx = 0; dev_idx < LTC_Get_Num_Devices(); dev_idx++) {
      // Assuming data format is [cell voltage, cell voltage, ..., PEC, PEC]
      // PEC for each device is the last two bytes of its data segment
      uint8_t *data_ptr = &read_voltages_reg[dev_idx * REG_LEN];
      uint16_t calculated_pec = LTC_PEC15_Calc(REG_LEN - 2, data_ptr); // Calculate PEC based on received data

      // Convert received PEC from two bytes to uint16_t
      uint16_t received_pec = (data_ptr[REG_LEN - 2] << 8) | data_ptr[REG_LEN - 1];

      if (received_pec == calculated_pec) {
        // If PEC matches, copy the voltage data, omitting the PEC bytes
        memcpy(&read_voltages[dev_idx * LTC_Get_Num_Series_Groups() + i * LTC_SERIES_GROUPS_PER_RDCV], data_ptr, REG_LEN - 2);
      } else {
        // Handle PEC mismatch error
        ret |= LTC_SPI_RX_ERROR;
      }
    }

    LTC_nCS_High(); //Pull CS high
  }

  return ret;
}

/*
Starts cell voltage conversion
*/
void LTC_ADCV(
  uint8_t MD, //ADC Mode
  uint8_t DCP, //Discharge Permit
  uint8_t CH //Cell Channels to be measured
) {
  uint8_t cmd[4];
  uint16_t cmd_pec;
  uint8_t md_bits;

  md_bits = (MD & 0x02) >> 1;
  cmd[0] = md_bits + 0x02;
  md_bits = (MD & 0x01) << 7;
  cmd[1] =  md_bits + 0x60 + (DCP<<4) + CH;
  cmd_pec = LTC_PEC15_Calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);


  LTC_Wakeup_Idle(); //This will guarantee that the ltc6811 isoSPI port is awake. This command can be removed.
  LTC_nCS_Low();
  HAL_SPI_Transmit(&hspi1, (uint8_t *)cmd, 4, 100);
  LTC_nCS_High();
}


int32_t LTC_PollAdc()
{
  uint32_t counter = 0;
  uint8_t finished = 0;
  uint8_t current_time = 0;
  uint8_t cmd[4];
  uint16_t cmd_pec;

  cmd[0] = 0x07;
  cmd[1] = 0x14;
  cmd_pec = LTC_PEC15_Calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  LTC_Wakeup_Idle(); //This will guarantee that the ltc6811 isoSPI port is awake. This command can be removed.

  LTC_nCS_Low();
  HAL_SPI_Transmit(&hspi1, (uint8_t *)cmd, 4, 100);

  while ((counter<200000)&&(finished == 0))
  {
    current_time = HAL_GetTick();
    if (current_time>0)
    {
      finished = 1;
    }
    else
    {
      counter = counter + 10;
    }
  }
  LTC_nCS_High();
  return(counter);
}

/* Read and store raw cell voltages at uint8_t 2d pointer */
int LTC_CalcPackVoltage(uint16_t *read_voltages) {
	int packvoltage = 0;
	for(int i = 0; i < LTC_Get_Num_Devices() * LTC_Get_Num_Series_Groups(); i++){
		packvoltage += read_voltages[i];
	}
	return packvoltage;
}
