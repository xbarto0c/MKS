/*
 * sct.h
 *
 *  Created on: 12. 10. 2022
 *      Author: xbarto0c
 */

#ifndef SCT_H_
#define SCT_H_


void sct_init(void); // initialize shift registers

void sct_led(uint32_t value); // display a value on the display

void sct_value(uint16_t value, uint8_t led);

#endif /* SCT_H_ */
