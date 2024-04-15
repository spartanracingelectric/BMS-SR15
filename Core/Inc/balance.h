#ifndef INC_BALANCE_H_
#define INC_BALANCE_H_

#include "main.h"

void Start_Balance(uint16_t *read_volt, uint8_t length, uint16_t lowest);
void End_Balance(uint8_t *faults);
void Discharge_Algo(uint16_t *read_volt, uint8_t total_ic, uint16_t lowest);
void Set_Cfg(uint8_t dev_idx, uint8_t *DCC);

#endif /* INC_BALANCE_H_ */
