/*
 * DC_Control.h
 *
 *  Created on: 21.03.2018
 *      Author: MT-LAB
 */

#ifndef DC_CONTROL_H_
#define DC_CONTROL_H_

#include "fram.h"
#include "sys.h"
#include "stdio.h"
#include "stdarg.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#define A_Enable 	GPIO_PIN_12
#define B_Enable 	GPIO_PIN_13

extern void DC_Control__25kHz();
extern void DC_Control__1kHz();

void SetPosition(float target_pos, float speed);

#endif /* DC_CONTROL_H_ */
