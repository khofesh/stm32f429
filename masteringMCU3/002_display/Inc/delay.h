/*
 * delay.h
 *
 *  Created on: Nov 11, 2023
 *      Author: fahmad
 */

#ifndef DELAY_H_
#define DELAY_H_

#include "stdint.h"

void delay_init(uint32_t freq);
uint32_t millis();
void delay(uint32_t time);

#endif /* DELAY_H_ */
