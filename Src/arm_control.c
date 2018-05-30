/*
 * DC_Control.c
 *
 *  Created on: 21.03.2018
 *      Author: MT-LAB
 */

/* USER CODE BEGIN Includes */
#include "arm_control.h"
#include "DMS.h"
/* USER CODE END Includes */

DataFrame  OutputDataFrame;

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


#define position_error 0.01f

typedef uint8_t bool;

typedef enum
{
	Weight, Prepare, Accelerate, Break, Stop
}Mode;

typedef enum{
	Neutral, Forward, Backward
}Direction;


float target_position = 0.25f;
float target_speed = 0;

Mode state = Accelerate;

void SetMotor(Direction direction, float Speed) {

	SetPWM_ChannelA(direction == Forward ? Speed : 0, direction == Backward ? Speed : 0 );
	HAL_GPIO_WritePin(GPIOB, A_Enable, 1 &&(direction == Forward || direction == Backward));

}

void shoot(float velocity, float angle) {
	if(velocity > 1) velocity = 1.0f;
		else if(velocity < 0) velocity = 0;

	if(angle > 0.25f) angle = 0.25f;
		else if(angle < 0) angle = 0;

	target_position = angle;
	target_speed = velocity;

	state = Prepare;
}

void weight_position() {
	state = Weight;
}

void arm_control__25kHz()
{
	//STATES
	static Mode				prev_state = Accelerate;	// previous state, used to detect state changes
	static uint32_t 		T_state;					// time spent in current state

	static int16_t 			EncoderOffset;

	static uint32_t 		T_UART;
	uint32_t UART_Interval 	= 10*ms;
	const uint32_t 			T = 1;


	//PARAMETERS


	//INPUTS
	int16_t EncoderRAW 			= (int16_t)htim4.Instance->CNT;
	//TODO: Initialize and correctly normalize encoder position
	float EncoderPosition  		= ((float)EncoderRAW)/8000;


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
		/*
		 * In the prepare state the arm is slowly moved down. Then it goes into the accelerate state
		 */
		case Prepare:
		{
			if(EncoderPosition > -0.125f) {
				Speed = 0.3;
				direction = Backward;
			} else if(EncoderPosition < -0.150f){
				Speed = 0.3;
				direction = Forward;
			} else {
				state = Accelerate;
			}
		}
		break;

		/*
		 * In the accelerate state the arm is accelerated to the target velocity.
		 * If the shooting position is reached it transitions into the break state.
		 */
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

		/*
		 * Breaks down the arm as fast as possible. Then transitions into the stop state.
		 */
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
			break;
		}

		case Stop:
		{
			Speed = 0;
			direction = Neutral;
		}

	}

	SetMotor(direction, Speed);


	//SetPWM_ChannelB(DMSPeriod/65535.0f,0);








}


void arm_control__1kHz()
{



}

