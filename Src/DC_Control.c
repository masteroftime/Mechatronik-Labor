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

typedef uint8_t bool;

typedef enum
{
	Init1, Init2, Operation
}Mode;




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

	//INPUTS
	float Current 	= (ADC1_value / 4095.0f * 3.3f) / 1.5;
	float tau 		= 20*ms;
	float dt 		= 1;
	float newPart 	= dt / tau;

	CurrentFiltered 			= CurrentFiltered * (1-newPart) + newPart * Current; //Tiefpass 1 Ordnung
	bool CurrentPeakDetected 	= CurrentFiltered > 0.6;

	int16_t EncoderRAW 			= (int16_t)htim4.Instance->CNT;
	float EncoderPosition  		= state == Operation ? ((EncoderRAW - EncoderOffset) * 2.0f / EncoderWidth) : 0; //-0.5 ... 0.5 wenn am Beginn mittig ausgerichtet

	//OUTPUTS
	float A, B = 0;

	Counter++;

	if(m_state != state) T_state = 0;
	else 				 T_state = T_state + 1;
	m_state = state;

	switch (state)
	{

		case Init1:
		{
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
			static uint32_t Direction = 0; //Rechts
			const uint32_t interval = 1*sec;

			if((Time + 1) < interval) 	Time = Time + 1;
			else 						Time = Time + 1 - interval;

			float tau = 2*(Time *1.0f) / interval;
			float offset = 8.0f/12;

			if(Direction == 0 && EncoderPosition > 0.35){
				Direction = 1;
			}
			else if (Direction == 1 && EncoderPosition < -0.35)
			{
				Direction = 0;
			}
			// Ab 9V beginnt der Linearschlitten zu fahren
			if(Direction)
			{
				A = 0.5;//offset + tau / (1.0f-offset);
				B = 0;
			}
			else{
				A = 0;
				B = 0.5;//offset + (tau-1) / (1.0f-offset);
			}
		}
		break;

	}

	SetPWM(A, B, 0, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, CurrentPeakDetected);

	if(Counter % 5000 == 0)
	{
		char UART_TX_DATA[40];
		sprintf(UART_TX_DATA, "%5ld\n", (uint32_t)(CurrentFiltered*1000));
		HAL_UART_Transmit(&huart2, (unsigned char*)UART_TX_DATA,strlen(UART_TX_DATA),100);
	}



}







void DC_Control__1kHz()
{



}

