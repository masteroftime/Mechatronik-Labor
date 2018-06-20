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
ControlFrame InputControlFrame;

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
	Init, Prepare, Accelerate, Break, Stop, Weight
}Mode;

typedef enum{
	Neutral, Forward, Backward
}Direction;


//Position variables
float zero_position = 4 * DEG;
float prepare_position = -60 * DEG;
float shooting_position = 45 * DEG;
float target_speed = 18.456f;			// target final velocity in rad/sec


#define POSITION_STABLE_TIME (3	*sec)
ui8 position_stable = 0;

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

	shooting_position = angle;
	target_speed = velocity;

	state = Prepare;
}

float simple_pi_pos_control(float target_pos, float current_pos, const ui32 T, float factor_p, float factor_i) {
	static float integral = 0;

	//Clamp factors
	if(factor_p < 0) factor_p = 0;

	if(factor_i < 0) factor_i = 0;

	//P and I koefficients
	float Ki = 0.012 * factor_i;
	float Kp = 3.5 * factor_p;


	float error = target_pos - current_pos;

	integral += error;
	CLAMP(integral, -1.0f/Ki, 1.0f/Ki)

	float pwm = Kp * error + Ki * integral;
	CLAMP(pwm, -1.0f, 1.0f)


	static float prev_position = 0;
	static ui32 pos_unchanged = 0;

	if(prev_position == current_pos) {
		if((pos_unchanged += T) > POSITION_STABLE_TIME) {
			position_stable = 1;
			//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		}
	} else {
		prev_position = current_pos;
		pos_unchanged = 0;
		position_stable = 0;
	//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	}

	return pwm;
}


float velocity_control(float target_velocity, float current_velocity, float current_angle, float mass, const unsigned long T, ARM_DATA* debug_data) {
	const float Kp = 4;
	const float Ki = (10.0f*T)/sec;

	static float integral = 0;

	float u_out = (mass-M_ARM)*G_ERD*L_ARM*Math__sin(current_angle+PI/2)*R_MOTOR/KI_MOTOR+KU_MOTOR*current_velocity
			+ Kp * (target_velocity-current_velocity) + integral;


	if(u_out > -V_SUPPLY && u_out < V_SUPPLY) {
		integral += Ki * (target_velocity-current_velocity);
	} else {
		integral = 0;
	}

	CLAMP(u_out, -V_SUPPLY, V_SUPPLY)

	debug_data->Out.Controller_Integral = integral;
	debug_data->Out.Controller_Out = u_out;
	debug_data->Out.Current_Angle = current_angle;
	debug_data->Out.Current_Velocity = current_velocity;
	debug_data->Out.Target_Velocity = target_velocity;

	return u_out/V_SUPPLY;
}


void arm_control__1kHz(const unsigned long T, ARM_DATA* Data)
{
	ControlFrame PcFrame = InputControlFrame;
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

	float EncoderPosition  		= (EncoderAbsPosition*1.0f)/Period + zero_position;		//in Rotations
	float EncoderSpeed 			= EncoderDelta*1.0f*sec / (Period*T); //in Rotations/second

	//Filter for Speed Value
	const float Tau = 100*ms;
	static float EncoderSpeedFiltered;
	float FilterConstant = (T*1.0f/Tau);
	EncoderSpeedFiltered = (1-FilterConstant) * EncoderSpeedFiltered + FilterConstant * EncoderSpeed; //Speed in rpm/second

	//Filter for Speed Value
	const float Tau2 = 10*ms;
	static float EncoderSpeedFiltered_10ms;
	float FilterConstant2 = (T*1.0f/Tau2);
	EncoderSpeedFiltered_10ms = (1-FilterConstant2) * EncoderSpeedFiltered_10ms + FilterConstant2 * EncoderSpeed; //Speed in rpm/second


	Data->Out.EncoderPosition 		= EncoderAbsPosition;
	Data->Out.EncoderPostionFloat 	= EncoderPosition;
	Data->Out.EncoderSpeed 			= EncoderSpeed;
	Data->Out.EncoderSpeedFiltered 	= EncoderSpeedFiltered;
	Data->Out.EncoderSpeed10msFiltered 	= EncoderSpeedFiltered_10ms;


	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, PcFrame.TaraCommand ? GPIO_PIN_RESET : GPIO_PIN_SET);

	static int MeasuredDirection = 0;

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

	float SpeedHardware =  90000000.0f/(20.0f)/(ValidSpeed*Period)* MeasuredDirection; //in Rotations/second


	Data->Out.SpeedHardware = SpeedHardware;
	Data->Out.Voltage = ADC1_value/4095.0f * 3.3f * 11;

	//OUTPUTS
	Direction direction = Neutral;
	float Speed = 0;


	//Input Button
	static bool Button = 0;
	bool mButton = Button;
	Button = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13) == GPIO_PIN_RESET;
	bool ButtonEdge = Button && !mButton;
	bool ButtonFallingEdge = !Button && mButton;

	static bool ShootCommand = 0;
	bool mShootCommand = ShootCommand;
	ShootCommand = PcFrame.ShootCommand;

	bool ShootCommandEdge = ShootCommand && !mShootCommand;

	if(ShootCommandEdge)
	{
		target_speed = PcFrame.ShootSpeed; //PC value in rad/sec
		shooting_position = PcFrame.ShootPosition / (2 * PI); //PC value in rad
	}


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
				 //state = Prepare;
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

			float Tmax = 1*sec;
			float T_rel = T_state /Tmax;

			if(T_rel>1) T_rel = 1;	//clamp T_rel
			float targetPosition = prepare_position*T_rel;


			float pwm = simple_pi_pos_control(targetPosition, EncoderPosition, T, 2.0f, 1.5f);

			if(pwm < 0) {
				direction = Backward;
				pwm *= -1;
			} else {
				direction = Forward;
			}

			Speed = pwm;

			if(ButtonEdge || ShootCommandEdge) {
				state = Accelerate;
			}

			/*if(EncoderPosition > -0.125f) {
				Speed = 0.3;
				direction = Backward;
			} else if(EncoderPosition < -0.150f){
				Speed = 0.3;
				direction = Forward;
			} else {
				state = Accelerate;
			}*/
		}
		break;

		/*
		 * In the accelerate state the arm is accelerated to the target velocity.
		 * If the shooting position is reached it transitions into the break state.
		 */
		case Accelerate:
		{
			/*if(EncoderPosition < target_position) {
			//if(T_state < 500 * ms) {
				Speed = 1;
				direction = Forward;
			} else {
				Speed = 1;
				direction = Backward;
				state = Break;
			}*/

			float velocity;

			if(EncoderPosition < shooting_position) {

				// phi(t) = a * t^2 / 2
				// v(t) = a * t = a * sqrt( 2 phi(t) / a) = sqrt( 2 a phi(t))
				// t = sqrt(2 phi(t) / a)


				//velocity = target_speed;
				//float a = target_speed*target_speed / (2 * shooting_position-prepare_position);
				//velocity = sqrt( 2 * a * (EncoderPosition-prepare_position) < 0 ? 0.5f : (EncoderPosition-prepare_position));

				float T_rel = (EncoderPosition-prepare_position)/(shooting_position-prepare_position);


				velocity = (0.5f + 0.5f * T_rel)*target_speed;
			} else {
				velocity = 0;

				if(EncoderSpeedFiltered <= 0.01f) {
					state = Stop;
				}
			}

			float current_angle = EncoderPosition*2*PI;
			float current_velocity = EncoderSpeedFiltered_10ms*2*PI;

			float pwm = velocity_control(velocity, current_velocity, current_angle, 0, T, Data);

			if(pwm < 0) {
				direction = Backward;
				pwm *= -1;
			} else {
				direction = Forward;
			}

			Speed = pwm;
		}
		break;

		/*
		 * Breaks down the arm as fast as possible. Then transitions into the stop state.
		 */
		case Break:
		{
			if(EncoderPosition > (shooting_position - 0.1f)) {
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
			float pwm = simple_pi_pos_control(0, EncoderPosition,T,1.0f, 1.0f);

			if(pwm < 0) {
				direction = Backward;
				pwm *= -1;
			} else {
				direction = Forward;
			}

			Speed = pwm;

			if(position_stable) {
				DoTara();
				state = Weight;
			}


			if(ButtonEdge)
			{
				state = Prepare;
			}

			break;
		}

		case Weight:
		{
			float pwm = simple_pi_pos_control(0, EncoderPosition,T,1.0f,1.0f);

			if(pwm < 0) {
				direction = Backward;
				pwm *= -1;
			} else {
				direction = Forward;
			}

			Speed = pwm;

			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13) == GPIO_PIN_RESET || ShootCommandEdge) {
				state = Prepare;
			}

			break;
		}

	}


	Data->Out.State = state;

	SetMotor(direction, Speed);


	//SetPWM_ChannelB(DMSPeriod/65535.0f,0);








}


void arm_control__25kHz(const unsigned long T, ARM_DATA* Data)
{

}
