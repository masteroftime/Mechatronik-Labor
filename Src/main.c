
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "fram.h"
#include "sys.h"
#include "stdio.h"
#include "stdarg.h"
#include "DC_Control.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
volatile uint16_t blinkDelay = 1000;

size_t
snprintfcat(
    char* buf,
    size_t bufSize,
    char const* fmt,
    ...)
{
    size_t result;
    va_list args;
    size_t len = strnlen( buf, bufSize);

    va_start( args, fmt);
    result = vsnprintf( buf + len, bufSize - len, fmt, args);
    va_end( args);

    return result + len;
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//	if(huart == &huart2)
//	{
//		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
//		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
//	}
//}

void PrintBytes(char* PrintBuffer, uint32_t PrintBufferSize ,void* Address, uint32_t Length, char Delimiter)
{

	snprintf(PrintBuffer, PrintBufferSize, "HexDump %8p [%ld]\n", Address, Length);

	char* Data = (char*)Address;

	for(uint32_t i = 0; i < (Length>>2); i++)
	{
		snprintfcat(PrintBuffer,PrintBufferSize,"%02X%02X%02X%02X%c", Data[i],  Data[i+1],  Data[i+2],  Data[i+3], Delimiter);
	}
}


#define BufferSize sizeof(ControlFrame)
#define BufferCount 2
uint32_t BufferIndex = 0;
uint32_t ReadIndex = 0;
unsigned char UART_RX_DATA[BufferCount][BufferSize] = {0};




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	sysStartup();
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM10_Init();
  MX_TIM13_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2); //Start timer to to provide trigger for the adc conversion
  //HAL_ADC_Start_IT(&hadc1);

  HAL_ADC_Start_IT(&hadc1);
  //HAL_ADC_Start(&hadc1);


  HAL_UART_Receive_DMA(&huart2,UART_RX_DATA[BufferIndex], sizeof(UART_RX_DATA[0]));
  ReadIndex = BufferIndex;
  BufferIndex = (BufferIndex + 1) % BufferCount;

  uint32_t CycleCounter = 0;

  //PWM output Half Bridge Driver
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);

  //PWM Output Reserve
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_IC_Start(&htim3,TIM_CHANNEL_1);
  HAL_TIM_IC_Start(&htim3,TIM_CHANNEL_2);


  //Timer for 25kHz interrupt signal
  HAL_TIM_PWM_Start_IT(&htim13,TIM_CHANNEL_1);

  //Timer for 1kHz interrupt signal
  HAL_TIM_PWM_Start_IT(&htim10,TIM_CHANNEL_1);


  //PWM input period capture
  HAL_TIM_Base_Start(&htim8);
  HAL_TIM_IC_Start_IT(&htim8,TIM_CHANNEL_1);
  HAL_TIM_IC_Start(&htim8,TIM_CHANNEL_2);


 unsigned long cntvalue = __HAL_TIM_GET_COUNTER(&htim2);

  //Encoder Initialize
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL); //Encoder


  fram_init();
  char SystemID[SYSTEM_ID_STRING_SIZE];
  getSystemID(SystemID, sizeof(SystemID));

  //SetPosition(-0.7f, 0.8f, 0.0f);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 // htim1.Instance->CCR1;


  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	  uint32_t Encoder4 = htim4.Instance->CNT;


//Auskommentiert 11.4.18
//	  sprintf(UART_TX_DATA, "Encoder %lu ", Encoder4);
//	  //HAL_UART_Transmit_IT(&huart2,(unsigned char*)UART_TX_DATA,strlen(UART_TX_DATA));
//	  HAL_UART_Transmit(&huart2, (unsigned char*)UART_TX_DATA,strlen(UART_TX_DATA),100);



//
//	  uint32_t ADC_Value = HAL_ADC_GetValue(&hadc1);
//	  sprintf(UART_TX_DATA, "ADC %lu\r\n", ADC_Value);
//	  //HAL_UART_Transmit_IT(&huart2,(unsigned char*)UART_TX_DATA,strlen(UART_TX_DATA));
//	  HAL_UART_Transmit(&huart2, (unsigned char*)UART_TX_DATA,strlen(UART_TX_DATA),100);
////	  PrintBytes(UART_TX_DATA,sizeof(UART_TX_DATA), htim1.Instance,sizeof(TIM_TypeDef),' ');
////	  HAL_UART_Transmit(&huart2, (unsigned char*)UART_TX_DATA,strlen(UART_TX_DATA),100);


	 unsigned long PWMInputCounterValue = __HAL_TIM_GetCounter(&htim8);    //read TIM2 counter value


	  HAL_Delay(10); //1ms Wait



	  CycleCounter++;
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */


void SendDataViaUART(void* Buffer, uint32_t length)
{

}

void UART_DataReceived()
{
//	uint32_t Data = UART_RX_DATA[ReadIndex][0]<<0 | UART_RX_DATA[ReadIndex][1]<<8 | UART_RX_DATA[ReadIndex][2]<<16 | UART_RX_DATA[ReadIndex][3]<<24;
//	if(Data == 0x12345678)
//	{
//		memcpy(&InputControlFrame, UART_RX_DATA[ReadIndex],sizeof(ControlFrame));
//
//		 SetPosition(InputControlFrame.PositionSetpoint,InputControlFrame.SpeedSetpoint, InputControlFrame.LorenzPositionSetpoint );
//
//		 //SetLorenzAktuator(InputControlFrame.LorenzPositionSetpoint,1.0f-InputControlFrame.LorenzPositionSetpoint);
//	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
	if(huart->Instance == huart2.Instance)
	{
		HAL_UART_Receive_DMA(&huart2,UART_RX_DATA[BufferIndex], sizeof(UART_RX_DATA[0]));
		ReadIndex = BufferIndex;
		BufferIndex = (BufferIndex + 1) % BufferCount;

		UART_DataReceived();

		//HAL_UART_Transmit_DMA(&huart2, UART_RX_DATA[ReadIndex], sizeof(UART_RX_DATA[0]));

		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
		//HAL_UART_Transmit_DMA(&huart2, UART_RX_DATA, strlen(UART_RX_DATA));

	}

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
	if(huart->Instance == huart2.Instance)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
	}

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
