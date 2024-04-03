#include "safety.h"

// ! Fault Thresholds
#define PACK_HIGH_VOLT_FAULT	    410000
#define PACK_LOW_VOLT_FAULT         288000

#define CELL_HIGH_VOLT_FAULT	    43000
#define CELL_LOW_VOLT_FAULT		    30000

#define CELL_HIGH_TEMP_FAULT		60

// ! Warnings Thresholds
#define PACK_HIGH_VOLT_WARNING	    408500
#define PACK_LOW_VOLT_WARNING       300000

#define CELL_HIGH_VOLT_WARNING	    42500
#define CELL_LOW_VOLT_WARNING	    32000

#define CELL_HIGH_TEMP_WARNING		55
#define CELL_LOW_TEMP_WARNING		0

#define CELL_VOLT_IMBALANCE_FAULT   5000
#define CELL_VOLT_IMBALANCE_WARNING	1000 //0.1 V

void Cell_Summary(struct batteryModule *batt) {
	batt->cell_volt_highest = batt->cell_volt[0];
	batt->cell_volt_lowest = batt->cell_volt[0];
	batt->cell_temp_highest = batt->cell_temp[0];
	batt->cell_temp_lowest = batt->cell_temp[0];
	batt->pack_voltage = 0;

	for (int i = 1; i < NUM_CELLS; i++) {

		if (batt->cell_volt[i] > batt->cell_volt_highest) {
			batt->cell_volt_highest = batt->cell_volt[i];
		}

		if (batt->cell_volt[i] < batt->cell_volt_lowest) {
			batt->cell_volt_lowest = batt->cell_volt[i];
		}

		batt->pack_voltage += batt->cell_volt[i];
	}

	for (int i = 0; i < NUM_THERM_TOTAL; i++) {
		if (batt->cell_temp_highest < batt->cell_temp[i]) {
			batt->cell_temp_highest = batt->cell_temp[i];
		}

		if (batt->cell_temp_lowest > batt->cell_temp[i]) {
			batt->cell_temp_lowest = batt->cell_temp[i];
		}
	}

}

void Fault_Warning_State(struct batteryModule *batt, uint8_t *fault,
		uint8_t *warnings, uint8_t *states) {

	if (batt->pack_voltage >= PACK_HIGH_VOLT_FAULT) {
		*fault |= 0b10000000;
	}

	if (batt->pack_voltage <= PACK_LOW_VOLT_FAULT) {
		*fault |= 0b01000000;
	}

	if (batt->cell_volt_lowest <= CELL_LOW_VOLT_FAULT) {
		*fault |= 0b00100000;
	}

	if (batt->cell_volt_highest >= CELL_HIGH_VOLT_FAULT) {
		*fault |= 0b00010000;
	}

	if (batt->cell_temp_highest >= CELL_HIGH_TEMP_FAULT) {
		*fault |= 0b00001000;
	}

	if ((batt->cell_volt_highest - batt->cell_volt_lowest)
			>= CELL_VOLT_IMBALANCE_FAULT) {
		*fault |= 0b00000100;
	}

	if (batt->pack_voltage >= PACK_HIGH_VOLT_WARNING) {
		*warnings |= 0b10000000;
	}

	if (batt->pack_voltage <= PACK_LOW_VOLT_WARNING) {
		*warnings |= 0b01000000;
	}

	if (batt->cell_volt_lowest <= CELL_LOW_VOLT_WARNING) {
		*warnings |= 0b00100000;
	}

	if (batt->cell_volt_highest >= CELL_HIGH_VOLT_WARNING) {
		*warnings |= 0b00010000;
	}

	if (batt->cell_temp_highest >= CELL_HIGH_TEMP_WARNING) {
		*warnings |= 0b00001000;
	}

	if (batt->cell_temp_lowest <= CELL_LOW_TEMP_WARNING) {
		*warnings |= 0b00000100;
	}

	if ((batt->cell_volt_highest - batt->cell_volt_lowest)
			>= CELL_VOLT_IMBALANCE_WARNING) {
		*warnings |= 0b00000010;
	}

	if (BALANCE) {
		*states |= 0b10000000;
	}
}

