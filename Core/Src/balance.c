/*
 * balance.c
 *
 *  Created on: Mar 8, 2024
 *      Author: hari
 */

#include "balance.h"
#include <math.h>
#include "print.h"
#include "6811.h"



/**
 * perform balance
 * 
 * @param read_volt array containing cells volts. 
 * @param length count of readings. 
 * @param lowest read_volt's lowest cell reading
*/
void balance_algorithm(uint16_t *read_volt, uint8_t length, uint16_t lowest) {
    
    
    for (int cell_index = 0; cell_index < length; cell_index++) {

        // check if each cell is close within 50 mV of the lowest cell. 

        if (read_volt[cell_index] - lowest > 50) {
            // differnce between cell and lowest cell is above 50 mV -- discharge it. 

            // TODO make a struct that contains DCC bits globally changeable. 


        }


    }
}