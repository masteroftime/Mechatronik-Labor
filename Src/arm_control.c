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
	Init, Prepare, Accelerate, Break, Stop
}Mode;

typedef enum{
	Neutral, Forward, Backward
}Direction;


float target_position = 0.25f;
float target_speed = 0;

Mode state = Init;

/*ui16 time1_overflow = 0;
ui16 time2_overflow = 0;
ui8 times_unchanged = 1;*/

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


void arm_control__25kHz(const unsigned long T, ARM_DATA* Data)
{
	//STATES
	static Mode				prev_state = Accelerate;	// previous state, used to detect state changes
	static uint32_t 		T_state;					// time spent in current state


	//Encoder Readout + Speed Calculation
	const uint16_t 	Period 				= 8640;
	static int32_t  EncoderAbsPosition 	= 0;	//Absolute Position Value in Steps - Can Handle multiple rotations
	int32_t 		EncoderDelta 		= 0;	//Difference Steps since last cycle

	static uint16_t EncoderRAW 	= 0; 			//This Cycle Register Value
	uint16_t mEncoderRAW 		= EncoderRAW; 	//Last Cycle Register Value
	EncoderRAW 					= htim4.Instance->CNT;

	int16_t Delta 				= EncoderRAW - mEncoderRAW;	//Register Differnce since last Cycle
	if		(Delta >  (int32_t)Period/2)	{ EncoderDelta = Delta - Period; EncoderAbsPosition += EncoderDelta; }
	else if (Delta < -(int32_t)Period/2)	{ EncoderDelta = Delta + Period; EncoderAbsPosition += EncoderDelta; }
	else 									{ EncoderDelta = Delta; 		 EncoderAbsPosition += EncoderDelta; }

	float EncoderPosition  		= (EncoderAbsPosition*1.0f)/Period;		//in Rotations
	float EncoderSpeed 			= EncoderDelta*1.0f*sec / (Period*T); //in Rotations/second

	//Filter for Speed Value
	const float Tau = 100*ms;
	static float EncoderSpeedFiltered;
	float FilterConstant = (T*1.0f/Tau);
	EncoderSpeedFiltered = (1-FilterConstant) * EncoderSpeedFiltered + FilterConstant * EncoderSpeed; //Speed in rpm/second

	//Filter for Speed Value
	const float Tau2 = 10*ms;
	static float EncoderSpeedFiltered2;
	float FilterConstant2 = (T*1.0f/Tau2);
	EncoderSpeedFiltered2 = (1-FilterConstant2) * EncoderSpeedFiltered2 + FilterConstant2 * EncoderSpeed; //Speed in rpm/second


	Data->Out.EncoderPosition 		= EncoderAbsPosition;
	Data->Out.EncoderPostionFloat 	= EncoderPosition;
	Data->Out.EncoderSpeed 			= EncoderSpeed;
	Data->Out.EncoderSpeedFiltered 	= EncoderSpeedFiltered;
	Data->Out.EncoderSpeed10msFiltered 	= EncoderSpeedFiltered2;


	/*static int MeasuredDirection = 0;

	if(EncoderDelta > 0) MeasuredDirection = 1;
	if(EncoderDelta < 0) MeasuredDirection = -1;

	//Mesung der Geschwindigkeit - Time between pulses
	static ui16 Time1 = 0;
	static ui16 Time2 = 0;

	static ui32 Tunchanged1 = 0;
	static ui32 Tunchanged2 = 0;

	ui16 mTime1 = Time1;
	ui16 mTime2 = Time2;
	Time1 = __HAL_TIM_GetCompare(&htim3, TIM_CHANNEL_1);
	Time2 = __HAL_TIM_GetCompare(&htim3, TIM_CHANNEL_2);


	ui16 Delta1 = Time1 - Time2;
	ui16 Delta2 = Time2 - Time1;

	if(Time1-mTime1 != 0) 	Tunchanged1 = 0;
	else 					Tunchanged1 += T;
	if(Time2-mTime2 != 0) 	Tunchanged2 = 0;
	else 					Tunchanged2 += T;

	static ui32 TimeInvalid = 0;
	ui32 OverflowTime = 40*ms;
	static bool Invalid = 1;
	bool mInvalid = Invalid;

	if(Invalid != mInvalid) 	TimeInvalid = 0;
	else 						TimeInvalid += T;


	if		(Tunchanged1 > OverflowTime || Tunchanged2 > OverflowTime) 										Invalid = 1;
	else if (Invalid && (TimeInvalid > 2) && (Tunchanged1 < OverflowTime) && (Tunchanged2 < OverflowTime)) 	Invalid = 0;
	//else																					Invalid = Invalid;

	float ValidSpeed = Invalid ? infinityf() :  MIN(Delta1, Delta2);
	//Prescaler 100-1

	ui32 timeDelta;

	float MeasuredSpeed =  90000000.0f/(100.0f)/(ValidSpeed*Period)* MeasuredDirection; //in Rotations/second


	Data->Out.DeltaTime = MeasuredSpeed;*/

	//OUTPUTS
	Direction direction = Neutral;
	float Speed = 0;


	// reset T_State if state has changed
	if(prev_state != state)  {
		T_state = 0;
		prev_state = state;
	} else {
		T_state += T;
	}

	switch (state)
	{
		case Init:
		{
			static bool mReferencePulse = 1;
			bool ReferencePulse 		= HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15);

			if(ReferencePulse && !mReferencePulse) //Positive Edge Detected
			{
				 __HAL_TIM_SET_COUNTER(&htim4,0);
				 EncoderRAW = 0;
				 EncoderAbsPosition = 0;
				 state = Stop;
			}
			mReferencePulse = ReferencePulse;

			Speed = 0.3;
			direction = Forward;

			break;
		}

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
			if(EncoderPosition > 0.005)
			{
				Speed = 0.3;
				direction = Backward;
			} else if(EncoderPosition < -0.005) {
				Speed = 0.3;
				direction = Forward;
			} else {
				Speed = 0;
				direction = Neutral;
			}
		}

	}

	SetMotor(direction, Speed);


	//SetPWM_ChannelB(DMSPeriod/65535.0f,0);








}


void arm_control__1kHz()
{



}
