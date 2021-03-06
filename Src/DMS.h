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
}DMS_IN;

typedef struct
{
	float DMSFiltered;
	float DMSOnTime;
	float DMSValueRaw;
	float DMSWeight;
}DMS_OUT;

typedef struct
{
	DMS_IN In;
	DMS_OUT Out;
}DMS_DATA;


void DMS_Update(uint32_t T, DMS_DATA* Data);

void DoTara();

extern unsigned long DMSPeriod;
extern unsigned long DMSDuty;


#endif /* DMS_H_ */
