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

    PRINTF("hello world.\r\n");

    CTIMER_StartTimer(CTIMER0_PERIPHERAL);
    CTIMER_StartTimer(CTIMER2_PERIPHERAL);


    Generate_sin_table_float(&SinTable_f[0], TABLE_LENGTH, 1.0f, 1.0f);

    while (1)
    {
        ch = GETCHAR();
        PUTCHAR(ch);
    }
}