/*
 * arm_control.h
 *
 *  Created on: 16.05.2018
 *      Author: MT-LAB
 */

#ifndef ARM_CONTROL_H_
#define ARM_CONTROL_H_

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

extern void arm_control__25kHz();
extern void arm_control__1kHz();


#endif /* ARM_CONTROL_H_ */
