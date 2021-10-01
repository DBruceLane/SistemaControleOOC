/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// StepperMotor
//#define stepsperrev 4096
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */



// StepperMotor
uint32_t iStep;

uint32_t MeioPeriodo = 30;   // MeioPeriodo no pulso em microsegundos  correcao de +10 ms 1490
uint32_t PPS = 200;          // Pulsos por segundo
//bool sentido = true;   // Variavel de sentido
uint32_t PPR = 200;            // Número de passos por volta
uint32_t Pulsos;               // Pulsos para o driver do motor
uint32_t Voltas;               // voltas do motor
//float RPM;                // Rotacoes por minuto
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

// StepperMotor
void rst_DRV8825()
{
  HAL_GPIO_WritePin(GPIOB, RST_Pin, 0);    // Realiza o reset do DRV8825
  HAL_Delay (1);                           // Atraso de 1 milisegundo
  HAL_GPIO_WritePin(GPIOB, RST_Pin, 1);    // Libera o reset do DRV8825
  HAL_Delay (10);                          // Atraso de 10 milisegundos
}

void disa_DRV8825()
{
  HAL_GPIO_WritePin(GPIOB, ENA_Pin, 1);    // Desativa o chip DRV8825
  HAL_Delay (10);                          // Atraso de 10 milisegundos
}

void ena_DRV8825()
{
  HAL_GPIO_WritePin(GPIOB, ENA_Pin, 0);     // Ativa o chip DRV8825
  HAL_Delay (10);                           // Atraso de 10 milisegundos
}

void HOR()                                    // Configura o sentido de rotação do Motor
{
  HAL_GPIO_WritePin(GPIOA, DIR_Pin, 1);       // Configura o sentido HORÁRIO
  
}

void AHR()                                    // Configura o sentido de rotação do Motor
{
  HAL_GPIO_WritePin(GPIOA, DIR_Pin, 0);       // Configura o sentido ANTI-HORÁRIO
  
}

void PASSO()                              // Pulso do passo do Motor
{
  HAL_GPIO_WritePin(GPIOB, STP_Pin, 0);   // Pulso nível baixo
  HAL_Delay (1);                          // MeioPeriodo de X milisegundos
  HAL_GPIO_WritePin(GPIOB, STP_Pin, 1);   // Pulso nível alto
  HAL_Delay (1);                          // MeioPeriodo de X milisegundos
}

void P1_32()
{
  PPR = 6400;                        // PPR pulsos por volta
  HAL_GPIO_WritePin(GPIOB, M0_Pin, 1);    // Configura modo Micro Passo 1/32
  HAL_GPIO_WritePin(GPIOB, M1_Pin, 2);
  HAL_GPIO_WritePin(GPIOB, M2_Pin, 1);
  
}

void TesteMotor()            // Gira motor nos dois sentidos
{
  HOR();
  uint32_t i;
  for (i = 0; i <= Pulsos; i++)       // Incrementa o Contador
  {
    PASSO();                              // Avança um passo no Motor
  }
  disa_DRV8825();
  HAL_Delay (750) ;                           // Atraso de 750 mseg
  ena_DRV8825();
  AHR();
  
  for (i = 0; i <= Pulsos; i++)       // Incrementa o Contador
  {
    PASSO();                              // Avança um passo no Motor
  }
  disa_DRV8825();
  HAL_Delay (750) ;                           // Atraso de 750 mseg
  ena_DRV8825();
}
void Frequencia()                     // Configura Frequencia dos pulsos
{
  Pulsos = PPR * Voltas;              // Quantidade total de Pulsos  PPR = pulsos por volta
  //PPS = 1000000 / (2 * MeioPeriodo);  // Frequencia Pulsos por segundo
  PPS = 2;
  //RPM = (PPS * 60) / PPR;             // Calculo do RPM
}
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */


  // StepperMotor
  
  disa_DRV8825();           // Desativa as saidas DRV8825
  P1_32();                    // Seleciona modo Passo Completo

  HAL_GPIO_WritePin(GPIOB, SLP_Pin, 0);  // Desativa modo sleep do DRV8825
  rst_DRV8825();            // Reseta o chip DRV8825
  HAL_GPIO_WritePin(GPIOB, ENA_Pin, 1);   // Ativa as saidas DRV8825

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Blinky LED
    HAL_GPIO_WritePin(GPIOC, LEDc13_Pin, 0);
    HAL_Delay(25);
    HAL_GPIO_WritePin(GPIOC, LEDc13_Pin, 1);
    HAL_Delay(50);

	  // StepperMotor
    Voltas = 1;         // Numero de voltas no Motor
    P1_32();            // Selecione o Modo do Passo FULL() HALF() P1_4() P1_8() P1_16() P1_32()
    Frequencia();       // Calcula RPM
    
    TesteMotor();       // Testa o Motor
	  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LEDc13_GPIO_Port, LEDc13_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, STP_Pin|SLP_Pin|RST_Pin|M2_Pin
                          |M1_Pin|M0_Pin|ENA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LEDc13_Pin */
  GPIO_InitStruct.Pin = LEDc13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LEDc13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIR_Pin */
  GPIO_InitStruct.Pin = DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STP_Pin SLP_Pin RST_Pin M2_Pin
                           M1_Pin M0_Pin ENA_Pin */
  GPIO_InitStruct.Pin = STP_Pin|SLP_Pin|RST_Pin|M2_Pin
                          |M1_Pin|M0_Pin|ENA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
