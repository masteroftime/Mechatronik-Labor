/*
 * DMS.h
 *
 *  Created on: 30.05.2018
 *      Author: MT-LAB
 */

#ifndef DMS_H_
#define DMS_H_

#include "System.h"





typedef struct
{
	uint32_t UART_Send;
}DMS_IN;

typedef struct
{

}DMS_OUT;



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

DMS_OUT DMS_Update(uint32_t T, DMS_IN IN);

extern unsigned long DMSPeriod;
extern unsigned long DMSDuty;


#endif /* DMS_H_ */
