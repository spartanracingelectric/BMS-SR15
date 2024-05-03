#include "balance.h"
#include "6811.h"

//DEFAULT VALUES THAT ARE SET IN CONFIG REGISTERS
//static int GPIO[5] = { 1, 1, 1, 1, 1 };
//static int REFON = 0;
//static int DTEN = 1; (READ ONLY BIT, we dont change it)
//static int ADCOPT = 0;
//static uint8_t VUV = 0x00;
//static uint8_t VOV_and_VUV = 0x00;
//static uint8_t VOV = 0x00;
//static int DCTO[4] = { 1, 1, 1, 1 };
static uint8_t config[8][6] =
		{ { 0xF8, 0x00, 0x00, 0x00, 0x00, 0x20 }, { 0xF8, 0x00, 0x00, 0x00,
				0x00, 0x20 }, { 0xF8, 0x00, 0x00, 0x00, 0x00, 0x20 }, { 0xF8,
				0x00, 0x00, 0x00, 0x00, 0x20 }, { 0xF8, 0x00, 0x00, 0x00, 0x00,
				0x20 }, { 0xF8, 0x00, 0x00, 0x00, 0x00, 0x20 }, { 0xF8, 0x00,
				0x00, 0x00, 0x00, 0x20 }, { 0xF8, 0x00, 0x00, 0x00, 0x00, 0x20 } };

static uint8_t defaultConfig[8][6] = { { 0xF8, 0x00, 0x00, 0x00, 0x00, 0x20 }, {
		0xF8, 0x00, 0x00, 0x00, 0x00, 0x20 }, { 0xF8, 0x00, 0x00, 0x00, 0x00,
		0x20 }, { 0xF8, 0x00, 0x00, 0x00, 0x00, 0x20 }, { 0xF8, 0x00, 0x00,
		0x00, 0x00, 0x20 }, { 0xF8, 0x00, 0x00, 0x00, 0x00, 0x20 }, { 0xF8,
		0x00, 0x00, 0x00, 0x00, 0x20 }, { 0xF8, 0x00, 0x00, 0x00, 0x00, 0x20 } };

void Start_Balance(uint16_t *read_volt, uint8_t length, uint16_t lowest) {
	Discharge_Algo(read_volt, NUM_DEVICES, lowest);
	Wakeup_Sleep();
	LTC6811_WRCFG(NUM_DEVICES, config);
}

void End_Balance(uint8_t *faults) {
	Wakeup_Sleep();
	LTC6811_WRCFG(NUM_DEVICES, defaultConfig);
	*faults |= 0b00000010;
}

/**
 * perform balance
 * 
 * @param read_volt array containing cells volts. 
 * @param length count of readings. 
 * @param lowest read_volt's lowest cell reading
 */
void Discharge_Algo(uint16_t *read_volt, uint8_t total_ic, uint16_t lowest) {

	for (uint8_t dev_idx = 0; dev_idx < NUM_DEVICES; dev_idx++) {
		// check if each cell is close within 0.005V of the lowest cell.
		uint8_t DCC[12];
		for (uint8_t cell_idx = 0; cell_idx < NUM_CELL_SERIES_GROUP;
				cell_idx++) {
			if (read_volt[dev_idx * NUM_CELL_SERIES_GROUP + cell_idx] - lowest
					> 50) {
				DCC[cell_idx] = 1;
			} else {
				DCC[cell_idx] = 0;
			}
		}
		Set_Cfg(dev_idx, (uint8_t*) DCC);
	}
}

/**
 * setting configuration registers
 *
 * @param device index
 * @param array of DCC bits
 */
void Set_Cfg(uint8_t dev_idx, uint8_t *DCC) {
	for (uint8_t cell_idx = 0; cell_idx < NUM_CELL_SERIES_GROUP; cell_idx++) {
		if (DCC[cell_idx]) {
			if (cell_idx < 8) {
				config[dev_idx][4] |= (1 << cell_idx);
			} else if (cell_idx >= 8) {
				config[dev_idx][5] |= (1 << (cell_idx - 8));
			}
		} else {
			if (cell_idx < 8) {
				config[dev_idx][4] &= (~(1 << cell_idx));
			} else {
				config[dev_idx][5] &= (~(1 << (cell_idx - 8)));
			}
		}
	}
}

