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

#define sec (25000)
#define ms (sec / 1000)

// CONSTANTS
#define PI			3.1415926f
#define G_ERD		9.81f
#define M_ARM		0.05f
#define L_ARM		0.12f
#define R_MOTOR		10.5f
#define KI_MOTOR	0.4062f
#define KU_MOTOR	0.3253f
#define V_SUPPLY	30

typedef unsigned char bool;
typedef unsigned long ui32;
typedef signed long i32;
typedef unsigned short ui16;
typedef signed short i16;
typedef unsigned char ui8;
typedef signed char i8;
typedef unsigned long long ui64;
typedef signed long long i64;

#define MIN(a,b) ((a)<(b) ? (a) : (b))
#define MAX(a,b)  ((a)>(b) ? (a) : (b))

#define CLAMP(var, min, max) if(var < (min)) var = (min); if(var > (max)) var = (max);

#define DEG (1.0f/360)

void Update25kHz();
void Update1kHz();

float Math__sin(float);

typedef struct
{
	uint32_t StartData;
	float WaageA;
	float WaageB;
	float WaageC;
	float WaageD;
	float Controller_Out;
	float Controller_Integral;
	float Current_Velocity;
	float Current_Angle;
	float Target_Velocity;
	float Constant_Test_Data;
	ui32 State;
}
DataFrame;

DataFrame  OutputDataFrame;



#endif /* SYSTEM_H_ */
