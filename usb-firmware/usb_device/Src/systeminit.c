/*
 * systeminit.c
 *
 *  Created on: Oct 28, 2023
 *      Author: fahmad
 */

#include <stdint.h>

#include "system_stm32f4xx.h"
#include "Helpers/logger.h"

LogLevel system_log_level = LOG_LEVEL_DEBUG;

/**
 * HCLK = 72 MHz
 * PLL: M = 4, N = 72, P = 2, Q = 3
 * AHB pre-scaler = 1
 * APB pre-scaler 1 = 2, APB pre-scaler 2 = 1
 * MCO1 pre-scaler = 2
 */
static void configure_clock()
{

}

void SystemInit(void)
{
	configure_clock();
}
