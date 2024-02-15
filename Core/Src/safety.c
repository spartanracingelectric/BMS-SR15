#include "safety.h"

void cellSummary(struct batteryModuleVoltage *batt){
	batt->cell_volt_highest = batt->cell_volt[0];
	batt->cell_volt_lowest = batt->cell_volt[0];
	batt->cell_temp_highest = batt->cell_temp[0];
	batt->cell_temp_lowest = batt->cell_temp[0];
	batt->pack_voltage = 0;

	for(int i = 1; i < NUM_CELLS; i++){

		if(batt->cell_volt[i] > batt->cell_volt_highest){
			batt->cell_volt_highest = batt->cell_volt[i];
		}

		if(batt->cell_volt[i] < batt->cell_volt_lowest){
			batt->cell_temp_lowest = batt->cell_volt[i];
		}

		batt->pack_voltage += batt->cell_volt[i];
	}

	for(int i = 0; i < NUM_THERM_TOTAL; i++){
		if(batt->cell_temp_highest < batt->cell_temp[i]){
			batt->cell_temp_highest = batt->cell_temp[i];
		}

		if(batt->cell_temp_lowest > batt->cell_temp[i]){
			batt->cell_temp_lowest = batt->cell_temp[i];
		}
	}

}


void fault_and_warning(struct batteryModuleVoltage *batt, uint8_t *fault, uint8_t *warnings){

	if(batt->pack_voltage >= PACK_HIGH_VOLT_FAULT){
		*fault |= 0b10000000;
	}

	if(batt->pack_voltage <= PACK_LOW_VOLT_FAULT){
		*fault |= 0b01000000;
	}

	if(batt->cell_volt_lowest <= CELL_LOW_VOLT_FAULT){
		*fault |= 0b00100000;
	}

		if(batt->cell_volt_highest>= CELL_HIGH_VOLT_FAULT){
		*fault |= 0b00010000;
	}

	if(batt->cell_temp_highest >= CELL_HIGH_TEMP_FAULT){
		*fault |= 0b00001000;
	}

	if(batt->pack_voltage >=  PACK_HIGH_VOLT_WARNING){
		*warnings |= 0b10000000;
	}

	if(batt->pack_voltage <= PACK_LOW_VOLT_WARNING){
		*warnings |= 0b01000000;
	}

	if(batt->cell_volt_lowest <= CELL_LOW_VOLT_WARNING){
		*warnings |= 0b00100000;
	}

	if(batt->cell_volt_highest >= CELL_HIGH_VOLT_WARNING){
		*warnings |= 0b00010000;
	}

	if(batt->cell_temp_highest >= CELL_HIGH_TEMP_WARNING){
		*warnings |= 0b00001000;
	}

	if(batt->cell_temp_lowest <= CELL_LOW_TEMP_WARNING){
		*warnings |= 0b00000100;
	}
}


