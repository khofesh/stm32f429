/*
 * systick.c
 *
 *  Created on: Nov 11, 2023
 *      Author: fahmad
 */

#include "stm32f4xx.h"
#include "delay.h"

#define	CTRL_ENABLE					(1U<<0)
#define CTRL_CLKSRC					(1U<<2)
#define CTRL_COUNTFLAG				(1U<<16)
#define CTRL_TICKINT				(1U<<1)

volatile uint32_t mil;

void delay_init(uint32_t freq)
{
	SysTick->LOAD  = (freq/1000) - 1;

	/* clear systick current value register */
	SysTick->VAL = 0;

	/* enable systick and select internal clksrc */
	SysTick->CTRL = CTRL_ENABLE | CTRL_CLKSRC ;

	/* enable systick interrupt */
	SysTick->CTRL  |= CTRL_TICKINT;
}

uint32_t millis()
{
	__disable_irq();
	uint32_t ml = mil;
	__enable_irq();

	return ml;
}

void delay(uint32_t time)
{
	uint32_t start = millis();
	while((millis() - start) < time);
}

void SysTick_Handler(void)
{
	mil++;
}
