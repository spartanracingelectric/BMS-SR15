#ifndef INC_SAFETY_H_
#define INC_SAFETY_H_

#include "main.h"

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

#define CELL_VOLT_IMBALANCE_WARNING	    500

void cellSummary(struct batteryModuleVoltage *batt);
void fault_and_warning(struct batteryModuleVoltage *batt, uint8_t *faults, uint8_t *warnings);


#endif /* INC_SAFETY_H_ */
