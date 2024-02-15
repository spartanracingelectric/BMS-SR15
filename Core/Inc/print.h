/*
 * print.h
 *
 *  Created on: Jan 26, 2024
 *      Author: karth
 */

#ifndef INC_PRINT_H_
#define INC_PRINT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "print.h"
#include "usb_device.h"
#include "string.h"

void print(uint8_t num, uint16_t *read_temp);

#ifdef __cplusplus
}
#endif

#endif /* __PRINT_H__ */

