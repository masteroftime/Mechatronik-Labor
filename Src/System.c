/*
 * System.c
 *
 *  Created on: 30.05.2018
 *      Author: MT-LAB
 */


#include "System.h"
#include "DMS.h"
#include "arm_control.h"
#include "adc.h"

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



	arm_control__1kHz(T, &Arm);
	DMS_Update(T, &Dms);


	SendTimer = (SendTimer + T)%(2*ms);
	if(SendTimer == 0)
	{
		OutputDataFrame.StartData = 0x12345678;
		OutputDataFrame.WaageA = Dms.Out.DMSValueRaw;
		OutputDataFrame.WaageB = Dms.Out.DMSWeight;
		OutputDataFrame.WaageC = Dms.Out.DMSFiltered;
		OutputDataFrame.WaageD = Arm.Out.Voltage;
		OutputDataFrame.Controller_Out = Arm.Out.Controller_Out;
		OutputDataFrame.Controller_Integral = Arm.Out.Controller_Integral;
		OutputDataFrame.Target_Velocity = Arm.Out.Target_Velocity;
		OutputDataFrame.Current_Angle = Arm.Out.Current_Angle;
		OutputDataFrame.Current_Velocity = Arm.Out.Current_Velocity;
		OutputDataFrame.EncoderSpeed10msFiltered = Arm.Out.EncoderSpeed10msFiltered;
		OutputDataFrame.EncoderPosition = Arm.Out.EncoderPostionFloat;
		OutputDataFrame.EncoderSpeedHardware = Arm.Out.SpeedHardware;
		OutputDataFrame.Constant_Test_Data = 42;
		OutputDataFrame.State = Arm.Out.State;
		HAL_UART_Transmit_DMA(&huart2, &OutputDataFrame, sizeof(OutputDataFrame));
	}
}



