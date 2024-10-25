/*
 * controls.h
 *
 *  Created on: Aug 31, 2024
 *      Author: shir
 */

#ifndef CONTROLS_H_
#define CONTROLS_H_

#include "stm32wbxx_ll_gpio.h"

#define member_size(type, member) (sizeof( ((type){0}).member ))

/*
 * @var Aggregate structure to store the input from all the controls on the board.
 *      These include the 2 slide switches, rotary pot, motion sensor, ambient light sensor
 *      and on-chip voltage and temperature sensors.
 */

typedef struct controlsRec {
	uint8_t slide_sw1; /* Switch position: 1-4 */
	uint8_t slide_sw2; /* Switch position: 1-4 */
	uint8_t sensitivity; /* Motion sensitivity setting: 0-255 */
	uint8_t motion; /* Motion detected */
	uint16_t log2illum; /* estimated illuminance, as 10 * log2(I) + 50 */
	uint16_t dbg;
	uint16_t adc_voltage;
	int16_t adc_temp; /* on-chip temperature sensor */
} controlsRec;

#define AMBIENT_RANGE (1 << (CHAR_BIT * member_size(struct controlsRec, sensitivity)))
#define MSENSITIVITY_RANGE  (1 << (CHAR_BIT * member_size(struct controlsRec, sensitivity)))

extern struct controlsRec controls;

void readBoardControls(ADC_HandleTypeDef *hadc1);
void motion_send_configuration(uint8_t threshold);

#endif /* CONTROLS_H_ */
