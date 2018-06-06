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

void Update25kHz();
void Update1kHz();



typedef struct
{
	uint32_t StartData;
	float WaageA;
	float WaageB;
	float WaageC;
	float WaageD;
}
DataFrame;

DataFrame  OutputDataFrame;



#endif /* SYSTEM_H_ */
