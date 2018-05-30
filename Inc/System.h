/*
 * System.h
 *
 *  Created on: 30.05.2018
 *      Author: MT-LAB
 */

#ifndef SYSTEM_H_
#define SYSTEM_H_

#include "main.h"
#include "tim.h"
#include "stm32f4xx_hal.h"
#include "usart.h"

#define sec (25000 * 1.0f)
#define ms (sec / 1000)

void Update25kHz();
void Update1kHz();

#endif /* SYSTEM_H_ */
