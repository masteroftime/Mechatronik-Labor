/*
 * DC_Control.c
 *
 *  Created on: 21.03.2018
 *      Author: MT-LAB
 */

/* USER CODE BEGIN Includes */
#include "arm_control.h"
/* USER CODE END Includes */


void SetPWM_ChannelA(float A, float B)
{
	uint16_t ARR_value = htim1.Instance->ARR;
	htim1.Instance->CCR1 = A * ARR_value;
	htim1.Instance->CCR2 = B * ARR_value;
}

void SetPWM_ChannelB(float C, float D){
	uint16_t ARR_value = htim1.Instance->ARR;

	 htim1.Instance->CCR3 = C * ARR_value;
	 htim1.Instance->CCR4 = D * ARR_value;
	}



#define sec (25000 * 1.0f)
#define ms (sec / 1000)
#define position_error 0.01f

typedef uint8_t bool;

typedef enum
{
	Accelerate, Break, Stop
}Mode;

typedef enum{
	Neutral, Forward, Backward
}Direction;


float target_position = 0.25f;
float target_speed = 0;

void SetMotor(Direction direction, float Speed) {

	SetPWM_ChannelA(direction == Forward ? Speed : 0, direction == Backward ? Speed : 0 );
	HAL_GPIO_WritePin(GPIOB, A_Enable, 1 &&(direction == Forward || direction == Backward));

}



void arm_control__25kHz()
{
	//STATES
	static Mode				prev_state = Accelerate;	// previous state, used to detect state changes
	static Mode 			state = Accelerate;			// current state
	static uint32_t 		T_state;					// time spent in current state

	static int16_t 			EncoderOffset;

	static uint32_t 		T_UART;
	uint32_t UART_Interval 	= 10*ms;

	//PARAMETERS


	//INPUTS

	//OUTPUTS
	Direction direction = Neutral;
	float Speed = 0;

	bool UART_Send  = 0;
	if(T_UART < UART_Interval) 	{ T_UART = T_UART + 1; 				UART_Send = 0; }
	else 						{ T_UART = T_UART + 1 - UART_Interval; UART_Send = 1; }


	// reset T_State if state has changed
	if(prev_state != state)  {
		T_state = 0;
		prev_state = state;
	} else {
		++T_state;
	}

	switch (state)
	{

		case Accelerate:
		{
			if(EncoderPosition < target_position) {
			//if(T_state < 500 * ms) {
				Speed = 1;
				direction = Forward;
			} else {
				Speed = 1;
				direction = Backward;
				state = Break;
			}
		}
		break;

		case Break:
		{
			if(EncoderPosition > (target_position - 0.1f)) {
			//if(T_state < 100 * ms) {
				Speed = 1;
				direction = Backward;
			} else {
				Speed = 0;
				direction = Neutral;
				state = Stop;
			}
		}

		case Stop:
		{
			Speed = 0;
			direction = Neutral;
		}

	}

	SetMotor(direction, Speed);


	/*if(UART_Send)
	{
		OutputStatusFrame.LorenzPosition = 0;
		OutputStatusFrame.CurrentSpeed = 0;
		OutputStatusFrame.CurrentPosition = EncoderPosition;
		OutputStatusFrame.StartData = 0x12345678;
		HAL_UART_Transmit_DMA(&huart2, &OutputStatusFrame, sizeof(OutputStatusFrame));
	}*/

}


void arm_control__1kHz()
{



}

