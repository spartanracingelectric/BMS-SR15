#include "safety.h"
#include "usbd_cdc_if.h"


void cellSummary(struct batteryModuleVoltage *batt){
	char *data = "Cell Summary";
	*batt->cell_volt_highest = batt->cell_volt[0];
	*batt->cell_volt_lowest = batt->cell_volt[0];
	*batt->cell_temp_highest = batt->cell_temp[0];
	*batt->cell_temp_lowest = batt->cell_temp[0];

	for(int i = 1; i < NUM_CELLS; i++){

		if(batt->cell_volt[i] > *batt->cell_volt_highest){
			*batt->cell_volt_highest = batt->cell_volt[i];
//			CDC_Transmit_FS((uint8_t *) data, (size_t)strlen(data));
		}

		if(batt->cell_volt[i] < *batt->cell_volt_lowest){
			*batt->cell_temp_lowest = batt->cell_volt[i];
		}

		*batt->pack_voltage += batt->cell_volt[i];
	}

	for(int i = 0; i < NUM_THERM_TOTAL; i++){
		if(*batt->cell_temp_highest < batt->cell_temp[i]){
			*batt->cell_temp_highest = batt->cell_temp[i];
		}

		if(*batt->cell_temp_lowest > batt->cell_temp[i]){
			*batt->cell_temp_lowest = batt->cell_temp[i];
		}
	}

}


void fault_and_warning(struct batteryModuleVoltage *batt, uint16_t *checker){
	char *data1 = "Fault Detected";
	if(*batt->pack_voltage >= PACK_HIGH_VOLT_FAULT){
		*checker |= 0b1000000000000000;
	}

	if(*batt->pack_voltage <= PACK_LOW_VOLT_FAULT){
		*checker |= 0b0100000000000000;
	}

	if(*batt->cell_volt_lowest <= CELL_LOW_VOLT_FAULT){
		*checker |= 0b0010000000000000;
		CDC_Transmit_FS((uint8_t *) data1, (size_t)strlen(data1));
	}

	if(*batt->cell_volt_highest>= CELL_HIGH_VOLT_FAULT){
		*checker|= 0b0001000000000000;
	}

	if(*batt->cell_temp_highest >= CELL_HIGH_TEMP_FAULT){
		*checker |= 0b0000100000000000;
	}

	if(*batt->pack_voltage >=  PACK_HIGH_VOLT_WARNING){
		*checker |= 0b0000010000000000;
	}

	if(*batt->pack_voltage <= PACK_LOW_VOLT_WARNING){
		*checker |= 0b0000001000000000;
	}

	if(*batt->cell_volt_lowest <= CELL_LOW_VOLT_WARNING){
		*checker |= 0b0000000100000000;
	}

	if(*batt->cell_volt_highest >= CELL_HIGH_VOLT_WARNING){
		*checker|= 0b0000000010000000;
	}

	if(*batt->cell_temp_highest >= CELL_HIGH_TEMP_WARNING){
		*checker |= 0b0000000001000000;
	}

	if(*batt->cell_temp_lowest <= CELL_LOW_TEMP_WARNING){
		*checker |= 0b0000000000100000;
	}
}


