/*
 * LCD_Pins.h
 *
 *  Created on: Nov 11, 2023
 *      Author: fahmad
 */

#ifndef LCD_PINS_H_
#define LCD_PINS_H_

#include "delay.h"

void LCD_Pin_Init();
void LCD_SPI_Init();

// LCD
void LCD_RST();
void LCD_Write_Cmd(uint8_t cmd);
void LCD_Write_Data(uint8_t data);

#endif /* LCD_PINS_H_ */
