/*
 * DMS.c
 *
 *  Created on: 30.05.2018
 *      Author: MT-LAB
 */

#include "DMS.h"



unsigned long DMSPeriod;
unsigned long DMSDuty;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	 if (htim->Instance==TIM8)
	 {
		DMSDuty 	= __HAL_TIM_GetCompare(&htim8, TIM_CHANNEL_2);
		DMSPeriod 	= __HAL_TIM_GetCompare(&htim8, TIM_CHANNEL_1);    //read TIM2 channel 1 capture value
		__HAL_TIM_SetCounter(&htim8, 0);    //reset counter after input capture interrupt occurs
	 }
}



void DMS_Update(uint32_t T, DMS_DATA* Data)
{
	DMS_OUT OUT;
	//Waage
	const float Tau = 500*ms;
	static float DMSFiltered;
	float FilterConstant = (T*1.0f/Tau);
	DMSFiltered = (1-FilterConstant) * DMSFiltered + FilterConstant * DMSPeriod;


	const float TauTara = 500*ms;

	Data->Out.DMSFiltered = DMSFiltered;
	Data->Out.DMSOnTime = DMSDuty;

}

