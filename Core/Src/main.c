#include "main.h"
#include "stdio.h"

typedef void (*ptrf)(uint32_t dlyticks);
typedef void (*pfunction)(void);

I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;

uint8_t read_data[4];


#define FLASH_MAIN_APP_ADDR 0x8008000
#define FLASH_UPDATE_APP_ADDR 0x8010000
#define JUMP2UPD_FLAG_ADDR  0x0801F000
#define UPDMAIN_FLAG_ADDR 0x0801F004



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */



void go_to_main_app(void);
void go_to_update_app(void);
void update_main_app(void);


/* Private user code ---------------------------------------------------------*/

void go_to_main_app(void){
	uint32_t JumpAddress;
	pfunction Jump_To_Application;

	//check for code
	if (((*(uint32_t*) FLASH_MAIN_APP_ADDR) & 0x2FFE0000) == 0x20000000){
		__disable_irq();

		HAL_FLASH_Unlock();
		FLASH_PageErase(JUMP2UPD_FLAG_ADDR);
		CLEAR_BIT(FLASH->CR, (FLASH_CR_PER));
		HAL_FLASH_Lock();

		JumpAddress = *(uint32_t *) (FLASH_MAIN_APP_ADDR + 4);
		Jump_To_Application = (pfunction) JumpAddress;

		__set_MSP(*(uint32_t*) FLASH_MAIN_APP_ADDR);
		Jump_To_Application();
	}
	else {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	}
}


void go_to_update_app(void){
	uint32_t JumpAddress;
	pfunction Jump_To_Application;


	//check for code
	if (((*(uint32_t*) FLASH_UPDATE_APP_ADDR) & 0x2FFE0000) == 0x20000000){
		__disable_irq();

		//clear flag
		HAL_FLASH_Unlock();
		FLASH_PageErase(JUMP2UPD_FLAG_ADDR);
		CLEAR_BIT(FLASH->CR, (FLASH_CR_PER));
		HAL_FLASH_Lock();

		//jump to app
		JumpAddress = *(uint32_t *) (FLASH_UPDATE_APP_ADDR + 4);
		Jump_To_Application = (pfunction) JumpAddress;

		__set_MSP(*(uint32_t*) FLASH_UPDATE_APP_ADDR);
		Jump_To_Application();
	}
	else {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	}
}

void update_main_app(void){
	uint32_t READ_ADDRESS  = 0x08010000;
	uint32_t WRITE_ADDRESS = 0x08008000;
	uint32_t result = 0;

	__disable_irq();

	// clear flag
	HAL_FLASH_Unlock();
	FLASH_PageErase(UPDMAIN_FLAG_ADDR);
	CLEAR_BIT(FLASH->CR, (FLASH_CR_PER));
	HAL_FLASH_Lock();


	// ERASE MAIN APPLICATION
	HAL_FLASH_Unlock();
	while(WRITE_ADDRESS < 0x08010000){
		FLASH_PageErase(WRITE_ADDRESS);
		CLEAR_BIT (FLASH->CR, (FLASH_CR_PER));
		WRITE_ADDRESS += 0x00000400; //1 PAGE
	}
	HAL_FLASH_Lock();

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);

	// WRITE MAIN APPLICATION
	HAL_FLASH_Unlock();
	WRITE_ADDRESS = 0x08008000;
	while(WRITE_ADDRESS < 0x08010000){
		for (int i = 0; i<4; i++){
			read_data[i] = *(uint8_t *) READ_ADDRESS;
			READ_ADDRESS++;
		}

		result = (read_data[3] << 24) | (read_data[2] << 16) | (read_data[1] << 8) | read_data[0];
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, WRITE_ADDRESS, result);
		CLEAR_BIT (FLASH->CR, (FLASH_CR_PG));
		WRITE_ADDRESS+= 0x04;

		result = 0;
	}

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_FLASH_Lock();
	__enable_irq();
}




int main(void)
{

  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  //MX_TIM2_Init();
  //MX_I2C1_Init();



  //read the flag
  HAL_FLASH_Unlock();
  read_data[0] = *(uint8_t *) JUMP2UPD_FLAG_ADDR;
  read_data[1] = *(uint8_t *) UPDMAIN_FLAG_ADDR;
  HAL_FLASH_Lock();

  if (read_data[1] == 0x01){
	  update_main_app();
  }

  // if there is a flag, go to update application
  //else goto main application
  if (read_data[0] == 0x01){
	  go_to_update_app();
  }
  else {
	  go_to_main_app();
  }

  while (1)
  {

  }
}



/* --------------------------------------------------------------------------------*/

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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 16;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
