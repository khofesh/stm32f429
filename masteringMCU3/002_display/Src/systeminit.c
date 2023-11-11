/*
 * systeminit.c
 *
 *  Created on: Nov 11, 2023
 *      Author: fahmad
 */

#include <stdint.h>

#include "system_stm32f4xx.h"
#include "stm32f4xx.h"

/**
 * HCLK = 168 MHz
 * PLL: M = 4, N = 168, P = 2, Q = 7
 * AHB pre-scaler = 1
 * APB1 pre-scaler = 4, APB2 pre-scaler = 2
 */
static void configure_clock()
{
	/**
	 * flash latency
	 *
	 * wait states: 5 WS (6 CPU cycles)
	 * because, `150 <HCLK ≤ 180` and `volate range 2.7 V - 3.6 V`
	 *
	 * Configure Flash prefetch, Instruction cache, Data cache and wait state
	 */
	FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS;

	/* enable HSE */
	RCC->CR |= RCC_CR_HSEON;

	/* wait until HSE is stable */
	while(!READ_BIT(RCC->CR, RCC_CR_HSERDY));

	/** over drive settings **/
	/* enable clock for PWR */
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	MODIFY_REG(
		PWR->CR,
		PWR_CR_VOS,
		_VAL2FLD(PWR_CR_VOS, 0x3)
	);
	/* activate over drive mode */
	PWR->CR |= PWR_CR_ODEN;
	/* wait for over drive ready */
	while(!READ_BIT(PWR->CSR, PWR_CSR_ODRDY));
	/* over drive switch enable */
	PWR->CR |= PWR_CR_ODSWEN;

	/**
	 * configure PLL
	 *
	 * RCC PLL configuration register (RCC_PLLCFGR)
	 * 		Bit 22 PLLSRC: Main PLL(PLL) and audio PLL (PLLI2S) entry clock source
	 * 		1: HSE oscillator clock selected as PLL and PLLI2S clock entry
	 */
	MODIFY_REG(
		RCC->PLLCFGR,
		RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLP | RCC_PLLCFGR_PLLQ | RCC_PLLCFGR_PLLSRC,
		_VAL2FLD(RCC_PLLCFGR_PLLM, 4) | _VAL2FLD(RCC_PLLCFGR_PLLN, 180) |
		_VAL2FLD(RCC_PLLCFGR_PLLP, 2) | _VAL2FLD(RCC_PLLCFGR_PLLQ, 7) |
		_VAL2FLD(RCC_PLLCFGR_PLLSRC, 1)
	);

	/**
	 * enable PLL module
	 */
	RCC->CR |= RCC_CR_PLLON;

	// waits until PLL is stable
	while(!READ_BIT(RCC->CR, RCC_CR_PLLRDY));

	// switches system clock to PLL
	MODIFY_REG(RCC->CFGR,
			RCC_CFGR_SW,
			_VAL2FLD(RCC_CFGR_SW, RCC_CFGR_SW_PLL)
	);

	// AHB pre-scaler = 1 (default, but let's be explicit)
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

	// APB1 pre-scaler = 4
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;

	// APB2 pre-scaler = 2
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;

	// waits until PLL is used
	while(READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

	// disable HSI
	CLEAR_BIT(RCC->CR, RCC_CR_HSION);

	/*Enable FPU*/
	SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));
}

void SystemInit(void)
{
	configure_clock();
}
