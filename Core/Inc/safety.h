#ifndef INC_SAFETY_H_
#define INC_SAFETY_H_

#include "main.h"

void Cell_Summary(struct batteryModule *batt);
void Fault_Warning_State(struct batteryModule *batt, uint8_t *faults, uint8_t *warnings, uint8_t *states);


#endif /* INC_SAFETY_H_ */
