#ifndef INC_SAFETY_H_
#define INC_SAFETY_H_

#include "main.h"

void cellSummary(struct batteryModuleVoltage *batt);
void faultAndWarning(struct batteryModuleVoltage *batt, uint8_t *faults, uint8_t *warnings);


#endif /* INC_SAFETY_H_ */
