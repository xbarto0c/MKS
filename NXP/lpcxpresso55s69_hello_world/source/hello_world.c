/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "peripherals.h"
#include "math.h"
#include "fsl_powerquad.h"

#include "fsl_power.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

char ch;
float Sample_f;
uint32_t index = 0;
#define TABLE_LENGTH 1000
#define PI 3.14159265358
#define USE_POWERQUAD 1
float SinTable_f[TABLE_LENGTH];

/*******************************************************************************
 * Prototypes
 ******************************************************************************/


void TimerIRQ(uint32_t flags)
{
	Sample_f = SinTable_f[index];
	index++;
	if(index >= TABLE_LENGTH) index = 0;
}

void Generate_sin_table_float(float *table, uint32_t length,float amplitude,
        float frequency)
    {
        	uint32_t index;
        	float result;
        	float theta, sample_time;
        	sample_time=1.0f/(frequency * (float)length);
        for(index=0;index < length;index++)
        {
        	theta= 2.0f * PI * frequency * sample_time * (float)index;
			#if USE_POWERQUAD
        	PQ_SinF32(&theta, &result);
        	table[index]=amplitude * result;
			#else
        	table[index]=amplitude * sin(theta);
			#endif
        }
    }

void operation()
{
	for(uint32_t i = 0; i < 32768; i++)
	 {
		asm volatile("nop");
	 }
	//return number1/number2;
}

uint8_t strcmp_safe(char input[], char correct_password[])
{
	uint8_t correct_flag = 0;
	for(uint8_t i = 0; i < (sizeof(input) / sizeof(char)); i++)
	{
		if(input[i] == correct_password[i])
		{
			 correct_flag++;
			 asm volatile("nop");
			 asm volatile("nop");
		}
		else
		{
			 correct_flag--;
			 //asm volatile("nop");
		}
	}
	/*for(uint8_t i = 0; i < rand() % 5000; i++)
	{
		asm volatile("nop");
	}*/
	if(correct_flag == 4) return 0;
	else return 1;
}



/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
int main(void)
{


    /* Init board hardware. */
    /* set BOD VBAT level to 1.65V */
    POWER_SetBodVbatLevel(kPOWER_BodVbatLevel1650mv, kPOWER_BodHystLevel50mv, false);
    /* attach main clock divide to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();
    BOARD_InitBootPeripherals();
    PQ_Init(POWERQUAD);
#if !defined(DONT_ENABLE_FLASH_PREFETCH)
    /* enable flash prefetch for better performance */
    SYSCON->FMCCR |= SYSCON_FMCCR_PREFEN_MASK;
#endif

    // PRINTF("hello world.\r\n");

    CTIMER_StartTimer(CTIMER0_PERIPHERAL);
    CTIMER_StartTimer(CTIMER2_PERIPHERAL);


    Generate_sin_table_float(&SinTable_f[0], TABLE_LENGTH, 1.0f, 1.0f);

    uint32_t DWT1, DWT2;
    DWT1 = DWT->CYCCNT;
    operation();
    DWT2 = DWT->CYCCNT;
    PRINTF("\r\nThe function took %d cycles to execute", DWT2 - DWT1);
    srand(0x0123);

    while (1)
    {
        //ch = GETCHAR();
        //PUTCHAR(ch);
    	char password_stored[20] = "1234";
    	char input[20];

    	PRINTF("\r\nEnter password: ");
    	SCANF("%s", input);

    	DWT1 = DWT->CYCCNT;

    	//uint8_t status = strcmp(input, password_stored);
    	uint8_t status = strcmp_safe(input, password_stored);

    	DWT2 = DWT->CYCCNT;
    	PRINTF("\r\nThe function took %d cycles to execute", DWT2 - DWT1);

    	PRINTF("\r\nInput: %s", input);
    	if(!status) PRINTF("\r\nCorrect password");
    	else PRINTF("\r\nIncorrect password");
    }
}





