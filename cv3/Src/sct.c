/*
 * sct.c
 *
 *  Created on: 12. 10. 2022
 *      Author: xbarto0c
 */
#include "sct.h"

void sct_init(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // enable clock on port B

	GPIOB->MODER |= GPIO_MODER_MODER3_0; // set corresponding pins as outputs
	GPIOB->MODER |= GPIO_MODER_MODER4_0;
	GPIOB->MODER |= GPIO_MODER_MODER5_0;
	GPIOB->MODER |= GPIO_MODER_MODER10_0;

	sct_led(0); // shifts zeroes into shift registers
	sct_noe(0); // activates shift register's outputs
}


void sct_led(uint32_t value)
{
	for(uint8_t i = 0; i < 8 * sizeof(value); i++) // cycle through all of the 32 bits of the variable value
	{
		sct_sdi(value & 1); // send LSB of the shifted "value" variable

		sct_clk(1); // generate impulse on the clk pin
		sct_clk(0);
		value >>= 1; // shift the variable "value" one position to the right
	}

	sct_nla(0); // generate impulse on the latch pin
	sct_nla(1);
}

void sct_value(uint16_t value)
{
	uint32_t reg = 0;
	reg |= reg_values[0][(value / 100) % 10];
	reg |= reg_values[1][(value / 10) % 10];
	reg |= reg_values[2][value % 10];
	sct_led(reg);
}










