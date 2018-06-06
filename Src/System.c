/*
 * System.c
 *
 *  Created on: 30.05.2018
 *      Author: MT-LAB
 */


#include "System.h"
#include "DMS.h"
#include "arm_control.h"

ARM_DATA Arm;
DMS_DATA Dms;

DataFrame  OutputDataFrame;

void Update25kHz()
{
	const uint32_t T = 1;


	arm_control__25kHz(T, &Arm );


}

void Update1kHz()
{
	const uint32_t T = 25;
	static uint32_t SendTimer;


	arm_control__1kHz();
	DMS_Update(T, &Dms);


	SendTimer = (SendTimer + T)%(30*ms);
	if(SendTimer == 0)
	{
		OutputDataFrame.StartData = 0x12345678;
		OutputDataFrame.WaageA = Arm.Out.EncoderSpeed10msFiltered;
		OutputDataFrame.WaageB = Arm.Out.EncoderPostionFloat;
		OutputDataFrame.WaageC = Arm.Out.DeltaTime;
		OutputDataFrame.WaageD = Arm.Out.EncoderSpeedFiltered;
		HAL_UART_Transmit_DMA(&huart2, &OutputDataFrame, sizeof(OutputDataFrame));
	}
}



