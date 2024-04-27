#ifndef INC_SAFETY_H_
#define INC_SAFETY_H_

#include "main.h"

void Cell_Summary(struct batteryModule *batt);
void Fault_Warning_State(struct batteryModule *batt, uint8_t *fault,
		uint8_t *warnings, uint8_t *states, uint8_t *low_volt_hysteresis,
		uint8_t *high_volt_hysteresis, uint8_t *cell_imbalance_hysteresis);
void Module_Averages(struct batteryModule *batt);

#endif /* INC_SAFETY_H_ */
