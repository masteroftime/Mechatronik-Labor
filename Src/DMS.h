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
	float DMSFiltered;
	float DMSOnTime;
}DMS_OUT;

typedef struct
{
	DMS_IN In;
	DMS_OUT Out;
}DMS_DATA;


void DMS_Update(uint32_t T, DMS_DATA* Data);

extern unsigned long DMSPeriod;
extern unsigned long DMSDuty;


#endif /* DMS_H_ */
