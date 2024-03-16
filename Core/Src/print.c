/*
 * print.c
 *
 *  Created on: Jan 26, 2024
 *      Author: karth
 */

#include "print.h"

/**
 * @brief method used to PRINT CURRENT VALUE ONLY. 
 * @param current_value value output from HAL_read input. 
*/
void print_curr(uint16_t current_value) {
	// TODO change the type of @param current_value, it does not match method call. 
    char out_buf[20];
    sprintf(out_buf, "Current: %u/10000\n", current_value);
    USB_Transmit(out_buf, strlen(out_buf));
}

/**
 * @brief method used to print out adc information. 
 * @param len length of array 
 * @param read_temp pointer to array containing info. 
*/
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
