/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint32_t cnt;
uint16_t PWM_Output[3];


float freq = 10;
float Vt[3];

float Vt_shift[3];

uint8_t bt_run,status_run;

uint32_t ADC_Buffer[2];
uint16_t ADC_Read[3];
float ADC_Voltage[3];

uint8_t EMF_Logic[3];

float deg_step[6] = {0,60,120,180,240,300};
uint8_t sector;


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
};
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
//
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  HAL_ADC_Start_DMA(&hadc1, ADC_Buffer, 3);

  HAL_TIM_Base_Start_IT(&htim6);





  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */




	  char buffer[64];
	  sprintf(buffer,"%d,%d,%d\n",PWM_Output[0],PWM_Output[1],PWM_Output[2]);

	  HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer),1000);




	  bt_run = HAL_GPIO_ReadPin(BT_RUN_GPIO_Port, BT_RUN_Pin);

	  if(!HAL_GPIO_ReadPin(BT_RUN_GPIO_Port, BT_RUN_Pin))
	  {
		  while(!HAL_GPIO_ReadPin(BT_RUN_GPIO_Port, BT_RUN_Pin));
		  status_run ^= 1;
	  }




  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{


	if(GPIO_Pin == IN1_Pin)
	 EMF_Logic[0] = HAL_GPIO_ReadPin(IN1_GPIO_Port, IN1_Pin);
	else if(GPIO_Pin == IN2_Pin)
	 EMF_Logic[1] = HAL_GPIO_ReadPin(IN2_GPIO_Port, IN2_Pin);
	else if(GPIO_Pin == IN3_Pin)
	 EMF_Logic[2] = HAL_GPIO_ReadPin(IN3_GPIO_Port, IN3_Pin);


	if(EMF_Logic[0] == 1 && EMF_Logic[1] == 0 && EMF_Logic[2]==0)
		sector = 0;
	else if(EMF_Logic[0] == 1 && EMF_Logic[1] == 1 && EMF_Logic[2]==0)
		sector = 1;
	else if(EMF_Logic[0] == 0 && EMF_Logic[1] == 1 && EMF_Logic[2]==0)
		sector = 2;
	else if(EMF_Logic[0] == 0 && EMF_Logic[1] == 1 && EMF_Logic[2]==1)
		sector = 3;
	else if(EMF_Logic[0] == 0 && EMF_Logic[1] == 0 && EMF_Logic[2]==1)
		sector = 4;
	else if(EMF_Logic[0] == 1 && EMF_Logic[1] == 0 && EMF_Logic[2]==1)
		sector = 5;



}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM6)
	{

		ADC_Read[0] = ADC_Buffer[0] & 0xFFFF;
		ADC_Read[1] = (ADC_Buffer[0] >> 16) & 0xFFFF;
		ADC_Read[2] = ADC_Buffer[1] & 0xFFFF;

		ADC_Voltage[0] = ADC_Read[0] * 0.8058608;
		ADC_Voltage[1] = ADC_Read[1] * 0.8058608;
		ADC_Voltage[2] = ADC_Read[2] * 0.8058608;

		  float time,omega;
		  time = cnt * 0.001;

		  omega = 2 * M_PI * freq * time;


		  cnt++;

		  Vt[0] = 500 * sinf(omega);
		  Vt[1] = 500 * sinf(omega+2.0944);
		  Vt[2] = 500 * sinf(omega-2.0944);

		  Vt_shift[0] = Vt[0] + 500;
		  Vt_shift[1] = Vt[1] + 500;
		  Vt_shift[2] = Vt[2] + 500;

		  for(int i=0;i<3;i++)
		  {
			  long buff_v = Vt_shift[i];
			  PWM_Output[i] = map(buff_v, 0, 1000, 0, 500);
		  }

		  if (status_run)
		  {
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_Output[0]);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_Output[1]);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_Output[2]);
		  }

	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
