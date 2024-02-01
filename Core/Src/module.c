/*
 * module.c
 *
 *  Created on: Jan 31, 2024
 *      Author: karth
 */
#include "module.h"
#include <math.h>
#include "print.h"
#include "6811.h"

uint8_t BMS_IC[12][6] = { { 0x69, 0x28, 0x0F, 0xF9, 0x7F, 0xF9 }, { 0x69, 0x28,
		0x0F, 0xE9, 0x7F, 0xF9 }, { 0x69, 0x28, 0x0F, 0xD9, 0x7F, 0xF9 }, {
		0x69, 0x28, 0x0F, 0xC9, 0x7F, 0xF9 }, { 0x69, 0x28, 0x0F, 0xB9, 0x7F,
		0xF9 }, { 0x69, 0x28, 0x0F, 0xA9, 0x7F, 0xF9 }, { 0x69, 0x28, 0x0F,
		0x99, 0x7F, 0xF9 }, { 0x69, 0x28, 0x0F, 0x89, 0x7F, 0xF9 }, { 0x69,
		0x08, 0x0F, 0xF9, 0x7F, 0xF9 }, { 0x69, 0x08, 0x0F, 0xE9, 0x7F, 0xF9 },
		{ 0x69, 0x08, 0x0F, 0xD9, 0x7F, 0xF9 }, { 0x69, 0x08, 0x0F, 0xC9, 0x7F,
				0xF9 } };

void getActualTemps(uint8_t index, uint16_t *actual_temp, uint16_t data) {
	static float scalar;
	static float steinhart;
	scalar = 30000.0f / (float) (data) - 1.0f;
	scalar = (float) ntcSeriesResistance / scalar;
	steinhart = scalar / (float) ntcNominal;               // (R/Ro)
	steinhart = log(steinhart);                           // ln(R/Ro)
	steinhart /= (float) ntcBetaFactor;                    // 1/B * ln(R/Ro)
	steinhart += 1.0f / ((float) ntcNominalTemp + 273.15f);      // + (1/To)
	steinhart = 1.0f / steinhart;                         // Invert
	steinhart -= 273.15f;    // convert to degree

	actual_temp[index] = steinhart;

}

void readVolt(uint16_t *read_volt) {
	LTC_Wakeup_Idle();
	LTC_ADCV(MD_7KHZ_3KHZ, DCP_DISABLED, CELL_CH_ALL);
	LTC_PollAdc();
	LTC_ReadRawCellVoltages((uint16_t*) read_volt);
	print(12, (uint16_t*) read_volt);
}

void readTemp(uint16_t tempindex, uint16_t *read_temp, uint16_t *read_auxreg) {
	LTC_Wakeup_Idle();
	ltc6811_wrcomm(NUM_DEVICES, BMS_IC[tempindex]);
	LTC_Wakeup_Idle();
	ltc6811_stcomm();
	//end sending to mux to read temperatures

	LTC_Wakeup_Idle();
	LTC_ADAX(MD_7KHZ_3KHZ, 1); //doing GPIO all conversion
	LTC_PollAdc();
	if (!LTC_ReadRawCellTemps((uint16_t*) read_auxreg)) // Set to read back all aux registers
			{
		uint16_t data = read_auxreg[0];
		//getActualTemps(tempindex, (uint16_t *) read_temp, data);
		read_temp[tempindex] = data;

		//start for printing over serial for voltages
		print(12, (uint16_t*) read_temp);
		//end for printing over serial for voltages
	}
}

