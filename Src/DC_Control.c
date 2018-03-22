/*
 * DC_Control.c
 *
 *  Created on: 21.03.2018
 *      Author: MT-LAB
 */

/* USER CODE BEGIN Includes */
#include "DC_Control.h"
/* USER CODE END Includes */


void SetPWM(float A, float B, float C, float D)
{
	uint16_t ARR_value = htim1.Instance->ARR;
	 htim1.Instance->CCR1 = A * ARR_value;
	 htim1.Instance->CCR2 = B * ARR_value;
	 htim1.Instance->CCR3 = C * ARR_value;
	 htim1.Instance->CCR4 = D * ARR_value;
}



#define sec (25000 * 1.0f)
#define ms (sec / 1000)
#define position_error 0.01f

typedef uint8_t bool;

typedef enum
{
	Init1, Init2, Operation
}Mode;

typedef enum{
	Neutral, Left, Right
}Direction;


float target_position = 0;
float target_speed = 0;

void SetMotor(Direction direction, float Speed) {

	SetPWM(direction == Left ? Speed : 0, direction == Right ? Speed : 0, 0, 0);
	HAL_GPIO_WritePin(GPIOB, A_Enable, direction == Left || direction == Right);

}

void SetPosition(float target_pos, float speed) {

	if(target_pos > 1) 			target_position = 1;
	else if(target_pos < -1) 	target_position = -1;
	else 						target_position = target_pos;


	if(speed > 1) 		target_speed = 1;
	else if(speed < 0) 	target_speed = 0;
	else 				target_speed = speed;

}

void DC_Control__25kHz()
{
	static uint32_t 		Counter = 0;
	//STATES
	static Mode				m_state = Init1;
	static Mode 			state = Init1;
	static uint32_t 		Time = 0;
	static float 			CurrentFiltered;
	static int16_t 			EncoderOffset;
	static int16_t 			EncoderWidth;
	volatile static int16_t EncoderLeft;
	volatile static int16_t EncoderRight;
	static uint32_t 		T_state;

	//PARAMETERS
	float Tau 		= 20*ms;
	float ObstacleCurrent = 0.6f;


	//INPUTS
	float Current 	= (ADC1_value / 4095.0f * 3.3f) / 1.5;

	float dt 		= 1;
	float newPart 	= dt / Tau;

	CurrentFiltered 			= CurrentFiltered * (1-newPart) + newPart * Current; //Tiefpass 1 Ordnung
	bool CurrentPeakDetected 	= CurrentFiltered > ObstacleCurrent;

	int16_t EncoderRAW 			= (int16_t)htim4.Instance->CNT;
	float EncoderPosition  		= state == Operation ? ((EncoderRAW - EncoderOffset) * 2.0f / EncoderWidth) : 0; //-0.5 ... 0.5 wenn am Beginn mittig ausgerichtet

	//OUTPUTS
	float A, B = 0;
	Direction direction = Neutral;
	float Speed = 0;

	Counter++;

	if(m_state != state) T_state = 0;
	else 				 T_state = T_state + 1;
	m_state = state;

	switch (state)
	{

		case Init1:
		{
			Speed = 0.5f;
			direction = Left;
			A = 0.5f;
			B = 0;

			if(CurrentPeakDetected && T_state > 200*ms)
			{
				EncoderLeft = EncoderRAW;
				state = Init2;
			}

		}
		break;

		case Init2:
		{
			Speed = 0.5f;
			direction = Right;
			A = 0;
			B = 0.5f;

			if(CurrentPeakDetected && T_state > 200*ms)
			{
				EncoderRight = EncoderRAW;
				EncoderWidth = (EncoderRight - EncoderLeft);
				EncoderOffset = (EncoderRight + EncoderLeft) / 2;
				state = Operation;
			}
		}
		break;

		case Operation:
		{
			if(EncoderPosition > (target_position + position_error)) {
				Speed = target_speed;
				direction = Left;
			} else if(EncoderPosition < (target_position - position_error)) {
				Speed = target_speed;
				direction = Right;
			} else {
				Speed = 0;
				direction = Neutral;
			}



//			if(Direction == 0 && EncoderPosition > 0.35){
//				Direction = 1;
//			}
//			else if (Direction == 1 && EncoderPosition < -0.35)
//			{
//				Direction = 0;
//			}
//			// Ab 9V beginnt der Linearschlitten zu fahren
//			if(Direction)
//			{
//				Speed = 0.5f;
//				direction = Left;
//				A = 0.5;//offset + tau / (1.0f-offset);
//				B = 0;
//			}
//			else{
//				Speed = 0.5f;
//				direction = Right;
//				A = 0;
//				B = 0.5;//offset + (tau-1) / (1.0f-offset);
//			}
		}
		break;

	}

	SetMotor(direction, Speed);
//
//	if(Counter % 5000 == 0)
//	{
//		char UART_TX_DATA[40];
//		sprintf(UART_TX_DATA, "%5ld \nabc\n", (uint32_t)(CurrentFiltered*1000));
//		HAL_UART_Transmit(&huart2, (unsigned char*)UART_TX_DATA,strlen(UART_TX_DATA),100);
//	}

}


void DC_Control__1kHz()
{



}

