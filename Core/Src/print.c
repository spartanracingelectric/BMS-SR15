/*
 * print.c
 *
 *  Created on: Jan 26, 2024
 *      Author: karth
 */

#include "print.h"

void print(uint8_t len, uint16_t *read_temp) {
	char buf[20];
	char out_buf[2048] = "";
	char char_to_str[2];
	char_to_str[0] = '\n';
	char_to_str[1] = '\0';

	for (uint8_t i = 0; i < len; i++) {
		sprintf(buf, "C%u:%u/10000", i + 1, read_temp[i]);
		strncat(out_buf, buf, 20);
		strncat(out_buf, char_to_str, 2);
	}
	strncat(out_buf, char_to_str, 2);
	USB_Transmit(out_buf, strlen(out_buf));
}
