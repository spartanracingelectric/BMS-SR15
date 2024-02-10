#ifndef INC_SAFETY_H_
#define INC_SAFETY_H_

#include "main.h"

// ! Fault Thresholds
#define PACK_HIGH_VOLT_FAULT	    410000
#define PACK_LOW_VOLT_FAULT         288000

#define CELL_HIGH_VOLT_FAULT	    4300
#define CELL_LOW_VOLT_FAULT		    3000

#define CELL_HIGH_TEMP_FAULT		60000

// ! Warnings Thresholds
#define PACK_HIGH_VOLT_WARNING	    408500
#define PACK_LOW_VOLT_WARNING       300000

#define CELL_HIGH_VOLT_WARNING	    4250
#define CELL_LOW_VOLT_WARNING	    3200

#define CELL_HIGH_TEMP_WARNING		55000
#define CELL_LOW_TEMP_WARNING		0000

#define CELL_VOLT_IMBALANCE_WARNING	    50

void cellSummary(struct batteryModuleVoltage *batt);
void fault_and_warning(struct batteryModuleVoltage *batt, uint16_t *checker);


#endif /* INC_SAFETY_H_ */
