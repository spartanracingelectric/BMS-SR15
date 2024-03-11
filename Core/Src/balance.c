/*
 * balance.c
 *
 *  Created on: Mar 8, 2024
 *      Author: hari
 */

#include "balance.h"
#include <math.h>
#include "print.h"
#include "6811.h"

//DEFAULT VALUES THAT ARE SET IN CONFIG REGISTERS
//static int GPIO[5] = { 1, 1, 1, 1, 1 };
//static int REFON = 0;
//static int DTEN = 0;
//static int ADCOPT = 0;
//static uint8_t VUV = 0x00;
//static uint8_t VOV_and_VUV = 0x00;
//static uint8_t VOV = 0x00;
//static int DCTO[4] = { 0, 0, 0, 0 };
static uint8_t config[8][6] =
		{ { 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00 }, { 0xF8, 0x00, 0x00, 0x00,
				0x00, 0x00 }, { 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00 }, { 0xF8,
				0x00, 0x00, 0x00, 0x00, 0x00 }, { 0xF8, 0x00, 0x00, 0x00, 0x00,
				0x00 }, { 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00 }, { 0xF8, 0x00,
				0x00, 0x00, 0x00, 0x00 }, { 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00 } };


void startBalance(uint16_t *read_volt, uint8_t length, uint16_t lowest) {
	wakeup_sleep();
	dischargeAlgo(read_volt, NUM_DEVICES, lowest);
	wakeup_idle();
	ltc6811_wrcfg(NUM_DEVICES, config);
}

/**
 * perform balance
 * 
 * @param read_volt array containing cells volts. 
 * @param length count of readings. 
 * @param lowest read_volt's lowest cell reading
 */
void dischargeAlgo(uint16_t *read_volt, uint8_t total_ic, uint16_t lowest) {

	for (uint8_t dev_idx = 0; dev_idx < NUM_DEVICES; dev_idx++) {
		// check if each cell is close within 50 mV of the lowest cell.
		uint8_t DCC[12];
		for (uint8_t cell_idx = 0; cell_idx < NUM_CELL_SERIES_GROUP; cell_idx++) {
			if (read_volt[dev_idx * NUM_CELL_SERIES_GROUP + cell_idx] - lowest
					> 50) {
				DCC[cell_idx] = 1;
			} else {
				DCC[cell_idx] = 0;
			}
		}
		setCfg(dev_idx, (uint8_t*) DCC);
	}
}

/**
 * setting configuration registers
 *
 * @param device index
 * @param array of DCC bits
 */
void setCfg(uint8_t dev_idx, uint8_t *DCC) {
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

