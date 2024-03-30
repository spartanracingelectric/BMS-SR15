#include "module.h"
#include <math.h>
#include "print.h"
#include "6811.h"

#define ntcNominal 10000.0f
#define ntcSeriesResistance 10000.0f
#define ntcBetaFactor 3435.0f
#define ntcNominalTemp 25.0f

static uint8_t BMS_THERM[][6] =
		{ { 0x69, 0x28, 0x0F, 0xF9, 0x7F, 0xF9 }, { 0x69, 0x28, 0x0F, 0xE9,
				0x7F, 0xF9 }, { 0x69, 0x28, 0x0F, 0xD9, 0x7F, 0xF9 }, { 0x69,
				0x28, 0x0F, 0xC9, 0x7F, 0xF9 }, { 0x69, 0x28, 0x0F, 0xB9, 0x7F,
				0xF9 }, { 0x69, 0x28, 0x0F, 0xA9, 0x7F, 0xF9 }, { 0x69, 0x28,
				0x0F, 0x99, 0x7F, 0xF9 },
				{ 0x69, 0x28, 0x0F, 0x89, 0x7F, 0xF9 }, { 0x69, 0x08, 0x0F,
						0xF9, 0x7F, 0xF9 },
				{ 0x69, 0x08, 0x0F, 0xE9, 0x7F, 0xF9 }, { 0x69, 0x08, 0x0F,
						0xD9, 0x7F, 0xF9 },
				{ 0x69, 0x08, 0x0F, 0xC9, 0x7F, 0xF9 }, { 0x69, 0x08, 0x0F,
						0xB9, 0x7F, 0xF9 },
				{ 0x69, 0x08, 0x0F, 0xA9, 0x7F, 0xF9 }, { 0x69, 0x08, 0x0F,
						0x99, 0x7F, 0xF9 },
				{ 0x69, 0x08, 0x0F, 0x89, 0x7F, 0xF9 } };

void getActualTemps(uint8_t dev_idx, uint8_t tempindex, uint16_t *actual_temp,
		uint16_t data) {
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

	actual_temp[dev_idx * NUM_THERM_PER_MOD + tempindex] = steinhart;

}

void readVolt(uint16_t *read_volt) {
	wakeup_idle();
	ltc_adcv(MD_7KHZ_3KHZ, DCP_DISABLED, CELL_CH_ALL);
	ltc_polladc();
	read_cell_volt((uint16_t*) read_volt);
}

void readTemp(uint8_t tempindex, uint16_t *read_temp, uint16_t *read_auxreg) {
	wakeup_idle();
	ltc_wrcomm(NUM_DEVICES, BMS_THERM[tempindex]);
	wakeup_idle();
	ltc_stcomm(2);
	//end sending to mux to read temperatures

	wakeup_idle();
	ltc_adax(MD_7KHZ_3KHZ, 1); //doing GPIO all conversion
	ltc_polladc();
	if (!read_cell_temps((uint16_t*) read_auxreg)) // Set to read back all aux registers
			{
		for (uint8_t dev_idx = 0; dev_idx < NUM_DEVICES; dev_idx++) {
			// Assuming data format is [cell voltage, cell voltage, ..., PEC, PEC]
			// PEC for each device is the last two bytes of its data segment
			uint16_t data = read_auxreg[dev_idx * NUM_AUX_GROUP];
			//read_temp[dev_idx * NUM_THERM_PER_MOD + tempindex] = data;
			getActualTemps(dev_idx, tempindex, (uint16_t*) read_temp, data); //+5 because vref is the last reg

		}
	}
}
