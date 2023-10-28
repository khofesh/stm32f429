/*
 * systeminit.c
 *
 *  Created on: Oct 28, 2023
 *      Author: fahmad
 */

#include <stdint.h>

#include "system_stm32f4xx.h"
#include "stm32f4xx.h"
#include "Helpers/logger.h"

LogLevel system_log_level = LOG_LEVEL_DEBUG;
uint32_t SystemCoreClock = 72000000; // 72 MHz

/**
 * HCLK = 72 MHz
 * PLL: M = 4, N = 72, P = 2, Q = 3
 * AHB pre-scaler = 1
 * APB pre-scaler 1 = 2, APB pre-scaler 2 = 1
 * MCO1 pre-scaler = 2
 */
static void configure_clock()
{
	/**
	 * CONFIGURE FLASH LATENCY
	 *
	 * reference manual
	 * Table 11. Number of wait states according to CPU clock (HCLK) frequency
	 * (STM32F42xxx and STM32F43xxx)
	 *
	 * and part `Increasing the CPU frequency`
	 *
	 * Wait states (WS) -> 2 WS
	 * 60 <HCLK ≤ 90
	 *
	 * (FLASH_ACR)
	 * Bits 3:0 LATENCY[3:0]: Latency
	 * 0010: Two wait states
	 */
	MODIFY_REG(FLASH->ACR,
			FLASH_ACR_LATENCY,
			//FLASH_ACR_LATENCY_2WS << FLASH_ACR_LATENCY_Pos
			_VAL2FLD(FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_2WS)
	);

	// enable HSE
	SET_BIT(RCC->CR, RCC_CR_HSEON);

	// waits until HSE is stable
	while(!READ_BIT(RCC->CR, RCC_CR_HSERDY));

	// configure PLL: source = HSE, SYSCLK = 72MHz
	MODIFY_REG(RCC->PLLCFGR,
			RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLP | RCC_PLLCFGR_PLLSRC | RCC_PLLCFGR_PLLQ,
			_VAL2FLD(RCC_PLLCFGR_PLLM, 4) | _VAL2FLD(RCC_PLLCFGR_PLLN, 72) | _VAL2FLD(RCC_PLLCFGR_PLLP, 0) | _VAL2FLD(RCC_PLLCFGR_PLLQ, 3) | _VAL2FLD(RCC_PLLCFGR_PLLSRC, 1)
	);

	// enable PLL module
	SET_BIT(RCC->CR, RCC_CR_PLLON);

	// waits until PLL is stable
	while(!READ_BIT(RCC->CR, RCC_CR_PLLRDY));

	// switches system clock to PLL
	MODIFY_REG(RCC->CFGR,
			RCC_CFGR_SW,
			_VAL2FLD(RCC_CFGR_SW, RCC_CFGR_SW_PLL)
	);

	// AHB pre-scaler = 1 (default, but let's be explicit)
	MODIFY_REG(RCC->CFGR,
			RCC_CFGR_HPRE,
			_VAL2FLD(RCC_CFGR_HPRE, 0)
	);

	// APB pre-scaler 1 = 2
	MODIFY_REG(RCC->CFGR,
			RCC_CFGR_PPRE1,
			_VAL2FLD(RCC_CFGR_PPRE1, 4)
	);

	// APB pre-scaler 2 = 1 (default)
	MODIFY_REG(RCC->CFGR,
			RCC_CFGR_PPRE2,
			_VAL2FLD(RCC_CFGR_PPRE2, 0)
	);

	// waits until PLL is used
	while(READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

	// disable HSI
	CLEAR_BIT(RCC->CR, RCC_CR_HSION);
}

void configure_mco1()
{
	// configure MC01 - source = PLL, MCO1PRE = 2
	MODIFY_REG(RCC->CFGR,
			RCC_CFGR_MCO1 | RCC_CFGR_MCO1PRE,
			_VAL2FLD(RCC_CFGR_MCO1, 3) | _VAL2FLD(RCC_CFGR_MCO1PRE, 4)
	);

	// enable GPIOA (MCO1 is connected to PA8).
	SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);

	// configure PA8 as medium speed
	MODIFY_REG(GPIOA->OSPEEDR,
			GPIO_OSPEEDR_OSPEED8,
			_VAL2FLD(GPIO_OSPEEDR_OSPEED8, 1)
	);

	// configure PA8 to work in alternate function mode
	MODIFY_REG(GPIOA->MODER,
			GPIO_MODER_MODER8,
			_VAL2FLD(GPIO_MODER_MODER8, 2)
	);
}

void SystemInit(void)
{
	configure_mco1();
	configure_clock();
}
