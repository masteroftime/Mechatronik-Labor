/*
 * arm_control.h
 *
 *  Created on: 16.05.2018
 *      Author: MT-LAB
 */

#ifndef ARM_CONTROL_H_
#define ARM_CONTROL_H_

#include "System.h"
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

//TYPES

typedef struct
{

}ARM_IN;

typedef struct
{
	i32 EncoderPosition;
	float EncoderPostionFloat;
	float EncoderSpeed;
	float EncoderSpeedFiltered;
	float EncoderSpeed10msFiltered;
	float SpeedHardware;
	ui32 Time2;
	float EncoderSpeedTime;
	float Voltage;
	float Controller_Out;
	float Controller_Integral;
	float Current_Velocity;
	float Current_Angle;
	float Target_Velocity;
}ARM_OUT;

typedef struct
{
	ARM_IN In;
	ARM_OUT Out;
}ARM_DATA;


//Functions, Variables

void arm_control__25kHz(const unsigned long T, ARM_DATA* Data);
void arm_control__1kHz(const unsigned long T, ARM_DATA* Data);

//void shoot();
void shoot(float, float);
uint8_t weight_ready();






#endif /* ARM_CONTROL_H_ */
