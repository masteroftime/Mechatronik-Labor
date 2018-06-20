/*
 * DMS.c
 *
 *  Created on: 30.05.2018
 *      Author: MT-LAB
 */

#include "DMS.h"
#include "arm_control.h"

//#define SCALING_FACTOR 51.337f
#define SCALING_FACTOR 18.88f

unsigned long DMSPeriod;
unsigned long DMSDuty;

float DMSFiltered;
float Tara = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	 if (htim->Instance==TIM8)
	 {
		DMSDuty 	= __HAL_TIM_GetCompare(&htim8, TIM_CHANNEL_2);
		DMSPeriod 	= __HAL_TIM_GetCompare(&htim8, TIM_CHANNEL_1);    //read TIM2 channel 1 capture value
		__HAL_TIM_SetCounter(&htim8, 0);    //reset counter after input capture interrupt occurs
	 }
}

void DoTara() {
	//Tara = DMSFiltered;
}


void DMS_Update(uint32_t T, DMS_DATA* Data)
{

	//Waage
	const float Tau = 100*ms;
	float FilterConstant = (T*1.0f/Tau);
	DMSFiltered = (1-FilterConstant) * DMSFiltered + FilterConstant * DMSPeriod;

	float HighPassRaw = DMSPeriod - DMSFiltered;
	const float TauHP = 20*ms;
	float FilterConstantHP = (T*1.0f/TauHP);
	float HighPassFiltered = (1-FilterConstantHP) * HighPassFiltered + FilterConstantHP * HighPassRaw;




	//Schmitt Trigger for the Weight Signal
	float ChangeWindowEnter = 6.5f;
	float ChangeWindowExit = 4;


	static bool TriggerSignal = 0;
	static ui32 TTrigger = 0;

	TTrigger += T;

	bool mTriggerSignal = TriggerSignal;
	if		(  !TriggerSignal && (TTrigger > 1*sec) && (HighPassFiltered < -ChangeWindowEnter || 0&& (HighPassFiltered > ChangeWindowEnter))) { TTrigger = 0; TriggerSignal = 1;}
	else if (	TriggerSignal && (HighPassFiltered > -ChangeWindowExit  && HighPassFiltered < ChangeWindowExit )) 						{ TTrigger = 0; TriggerSignal = 0;}
	//else TriggerSignal = TriggerSignal;

	bool TriggerEdge = TriggerSignal && !mTriggerSignal;
	bool TriggerFallingEdge = mTriggerSignal && !TriggerSignal;
	static ui32 TriggerCounter = 0;
	if(TriggerEdge) TriggerCounter++;

	static bool InWeightMode = 0;
	if(TriggerEdge) InWeightMode = 1;
	else if (TriggerFallingEdge) InWeightMode = 0;

	bool TaraButton = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3);

	bool TaraPc = InputControlFrame.TaraCommand != 0;
	if(TaraButton || 	TaraPc) Tara = DMSFiltered;


	Data->Out.DMSOnTime = DMSDuty;

	//Output
	Data->Out.DMSValueRaw = Tara;
	Data->Out.DMSWeight = (DMSFiltered - Tara)/SCALING_FACTOR;
	Data->Out.DMSFiltered = TriggerCounter;
}

