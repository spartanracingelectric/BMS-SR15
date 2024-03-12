#ifndef INC_BALANCE_H_
#define INC_BALANCE_H_

#include "main.h"

void startBalance(uint16_t *read_volt, uint8_t length, uint16_t lowest);
void dischargeAlgo(uint16_t *read_volt, uint8_t total_ic, uint16_t lowest);
void setCfg(uint8_t dev_idx, uint8_t *DCC);

#endif /* INC_BALANCE_H_ */
