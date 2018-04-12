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

typedef struct
{
	uint32_t StartData;
	float PositionSetpoint;
	float SpeedSetpoint;
	float LorenzPositionSetpoint;
}
ControlFrame;

typedef struct
{
	uint32_t StartData;
	float CurrentPosition;
	float CurrentSpeed;
	float LorenzPosition;

}StatusFrame;

ControlFrame InputControlFrame;
StatusFrame  OutputStatusFrame;

void SetLorenzAktuator(float A, float B);
void SetPosition(float target_pos, float speed, float lorenzInput);

#endif /* DC_CONTROL_H_ */
