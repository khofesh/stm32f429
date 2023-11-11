/*
 * LCD_Pins.c
 *
 *  Created on: Nov 11, 2023
 *      Author: fahmad
 *
 * PA7  -> RESX 	- LCD RST
 * PC2  -> CSX 		- LCD CS
 * PD13 -> WRX_D/CX - LCD DC
 * PF9  -> SDI/SDA 	- SPI5 SCK
 * PF7  -> D/CX_SCL - SPI5 MOSI
 */

#include "stm32f4xx.h"
#include "LCD_Pins.h"

#define AF05						0x05

#define LCD_RES_HIGH(void)			GPIOA->BSRR=GPIO_BSRR_BS7
#define LCD_RES_LOW(void)			GPIOA->BSRR=GPIO_BSRR_BR7

#define LCD_CS_HIGH(void)			GPIOC->BSRR=GPIO_BSRR_BS2
#define LCD_CS_LOW(void)			GPIOC->BSRR=GPIO_BSRR_BR2

#define LCD_DC_HIGH(void)			GPIOD->BSRR=GPIO_BSRR_BS13
#define LCD_DC_LOW(void)			GPIOD->BSRR=GPIO_BSRR_BR13

static void spi5_transmit(uint8_t *data, uint32_t size);

/**
 * Bits 2y:2y+1 MODERy[1:0]: Port x configuration bits (y = 0..15)
	These bits are written by software to configure the I/O direction mode.
	00: Input (reset state)
	01: General purpose output mode
	10: Alternate function mode
	11: Analog mode
 */
void LCD_Pin_Init()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN |
			RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOFEN;

	/* PA7 */
	GPIOA->MODER |= GPIO_MODER_MODE7_0;
	GPIOA->MODER &= ~GPIO_MODER_MODE7_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED7; // 11: Very high speed
	GPIOA->ODR |= GPIO_ODR_OD7;

	/* PC2 */
	GPIOD->MODER |= GPIO_MODER_MODE2_0;
	GPIOD->MODER &= ~GPIO_MODER_MODE2_1;
	GPIOD->OSPEEDR |= GPIO_OSPEEDR_OSPEED2; // 11: Very high speed
	GPIOD->ODR |= GPIO_ODR_OD2;

	/* PD13 */
	GPIOD->MODER |= GPIO_MODER_MODE13_0;
	GPIOD->MODER &= ~GPIO_MODER_MODE13_1;
	GPIOD->OSPEEDR |= GPIO_OSPEEDR_OSPEED13; // 11: Very high speed
	GPIOD->ODR |= GPIO_ODR_OD13;

	/**
	 * alternate function
	 * PF7 and PF9
	 */
	GPIOF->MODER |= GPIO_MODER_MODE7_1 | GPIO_MODER_MODE9_1;
	GPIOF->MODER &= ~(GPIO_MODER_MODE7_0 | GPIO_MODER_MODE9_0);
	GPIOF->AFR[0] |= (AF05 << GPIO_AFRL_AFSEL7_Pos);
	GPIOF->AFR[1] |= (AF05 << GPIO_AFRH_AFSEL9_Pos);
}

void LCD_SPI_Init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_SPI5EN;
	SPI5->CR1 |= SPI_CR1_MSTR|SPI_CR1_SSM|SPI_CR1_SSI|
			SPI_CR1_BR_1;

	SPI5->CR1 |= SPI_CR1_SPE;
}

static void spi5_transmit(uint8_t *data, uint32_t size)
{
	uint32_t i = 0;

	while(i < size)
	{
		/* wait until TXE is set */
		while(!READ_BIT(SPI5->SR, SPI_SR_TXE));

		/* write the data to the data register */
		SPI5->DR = data[i];
		i++;
	}

	while(!READ_BIT(SPI5->SR, SPI_SR_TXE));

	/* wait for BUSY flag to reset */
	while(READ_BIT(SPI5->SR, SPI_SR_BSY));

	/* clear ovr flag */
	(void)SPI5->DR;
	(void)SPI5->SR;
}

void LCD_RST()
{
	LCD_RES_LOW();
	delay(50);
	LCD_RES_HIGH();
	delay(20);
}

void LCD_Write_Cmd(uint8_t cmd)
{
	/* CS low - to accept SPI data */
	LCD_CS_LOW();
	/* DC low - command mode */
	LCD_DC_LOW();
	spi5_transmit(&cmd,1);
	/* CS high - end of transmission */
	LCD_CS_HIGH();
}

void LCD_Write_Data(uint8_t data)
{
	LCD_CS_LOW();
	LCD_DC_HIGH();
	spi5_transmit (&data,1);
	LCD_CS_HIGH();
}

