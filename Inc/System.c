/*
 * System.c
 *
 *  Created on: 30.05.2018
 *      Author: MT-LAB
 */


#include "System.h"



void Update25kHz()
{
	const uint32_t T = 1;


	arm_control__25kHz();


}

void Update1kHz()
{
	const uint32_t T = 25;


	arm_control__1kHz();
	DMS_Update(T);
}



