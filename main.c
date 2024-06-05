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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define QMC5883L_ADDR (0x0D << 1)
#define QMC5883L_REGISTER_1_ADDR 0x09 
#define QMC5883L_REGISTER_3_ADDR 0x0B 

#define LCD_I2C_ADDRESS (0x27 << 1)


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

void Buzzer_Active(void);
void QMC5883L_SetStandbyMode(void);
void QMC5883L_SetContinuousMode(void);
void QMC5883L_ReadData(int16_t* x, int16_t* y, int16_t* z);
void Caculate_AzimuthAngle(float* azimuth_angle, int16_t x, int16_t y);
void LCD_Init(void);
void LCD_SendCommand(uint8_t cmd);
void LCD_SendData(uint8_t data);
void LCD_SendString(char *str, uint8_t row, uint8_t col);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_ClearDisplay();


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int16_t x=0;
int16_t y=0;
int16_t z=0;
bool volatile is_system_on = false;
float azimuth_angle=0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == SW1_Pin){
		is_system_on = !is_system_on;
		if(is_system_on == true){
			__HAL_TIM_SET_AUTORELOAD(&htim2, 999);
			__HAL_TIM_SET_COUNTER(&htim2, 0);
			HAL_TIM_Base_Start_IT(&htim2);
			HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 1);
			HAL_TIM_Base_Start_IT(&htim1);
			QMC5883L_SetContinuousMode();
		}else{
			__HAL_TIM_SET_AUTORELOAD(&htim2, 499);
			__HAL_TIM_SET_COUNTER(&htim2, 0);
			HAL_TIM_Base_Start_IT(&htim2);
			HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, 1);
			HAL_TIM_Base_Stop_IT(&htim1);
			QMC5883L_SetStandbyMode();
		}
		Buzzer_Active();
		while(HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == 0){}
		HAL_Delay(30);
		__HAL_GPIO_EXTI_CLEAR_IT(SW1_Pin);
		HAL_NVIC_ClearPendingIRQ(SW1_EXTI_IRQn);
	}else if(GPIO_Pin == SW2_Pin){
		is_system_on = false;
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 1);
		HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, 1);
		HAL_TIM_Base_Stop_IT(&htim2);
		HAL_TIM_Base_Stop_IT(&htim1);
		QMC5883L_SetStandbyMode();
		LCD_ClearDisplay();
		Buzzer_Active();
		while(HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == 0){}
		HAL_Delay(30);
		__HAL_GPIO_EXTI_CLEAR_IT(SW2_Pin);
		HAL_NVIC_ClearPendingIRQ(SW2_EXTI_IRQn);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM1){
		QMC5883L_ReadData(&x, &y, &z);
		Caculate_AzimuthAngle(&azimuth_angle, x, y);
		LCD_ClearDisplay();
		LCD_SendString("Phuong huong", 0, 2);
		if (azimuth_angle > 348.75 || azimuth_angle <= 11.25) {
			LCD_SendString("Bac", 1, 6);
		}
		else if (azimuth_angle > 11.25 && azimuth_angle <= 33.75) {
			LCD_SendString("Bac Dong Bac", 1, 2);
		}
		else if (azimuth_angle > 33.75 && azimuth_angle <= 56.25) {
			LCD_SendString("Dong Bac", 1, 4);
		}
		else if (azimuth_angle > 56.25 && azimuth_angle <= 78.75) {
			LCD_SendString("Dong Dong Bac", 1, 1);
		}
		else if (azimuth_angle > 78.75 && azimuth_angle <= 101.25) {
			LCD_SendString("Dong", 1, 6);
		}
		else if (azimuth_angle > 101.25 && azimuth_angle <= 123.75) {
			LCD_SendString("Dong Dong Nam", 1, 1);
		}
		else if (azimuth_angle > 123.75 && azimuth_angle <= 146.25) {
			LCD_SendString("Dong Nam", 1, 4);
		}
		else if (azimuth_angle > 146.25 && azimuth_angle <= 168.75) {
			LCD_SendString("Nam Dong Nam", 1, 2);
		}
		else if (azimuth_angle > 168.75 && azimuth_angle <= 191.25) {
			LCD_SendString("Nam", 1, 6);
		}
		else if (azimuth_angle > 191.25 && azimuth_angle <= 213.75) {
			LCD_SendString("Nam Tay Nam", 1, 2);
		}
		else if (azimuth_angle > 213.75 && azimuth_angle <= 236.25) {
			LCD_SendString("Tay Nam", 1, 4);
		}
		else if (azimuth_angle > 236.25 && azimuth_angle <= 258.75) {
			LCD_SendString("Tay Tay Nam", 1, 2);
		}
		else if (azimuth_angle > 258.75 && azimuth_angle <= 281.25) {
			LCD_SendString("Tay", 1, 6);
		}
		else if (azimuth_angle > 281.25 && azimuth_angle <= 303.75) {
			LCD_SendString("Tay Tay Bac", 1, 2);
		}
		else if (azimuth_angle > 303.75 && azimuth_angle <= 326.25) {
			LCD_SendString("Tay Bac", 1, 4);
		}
		else if (azimuth_angle > 326.25 && azimuth_angle <= 348.75) {
			LCD_SendString("Bac Tay Bac", 1, 2);
		}
	}else if(htim->Instance == TIM2){
		if(is_system_on == true){
			HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
		}else{
			HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
		}
	}
}

void Buzzer_Active(){
	HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
	HAL_Delay(60);
	HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);
}

void QMC5883L_SetContinuousMode(){
  uint8_t config_3_value = 0x01;
  uint8_t config_1_value = 0x01 | 0x08 | 0x00 | 0x00;
    
  HAL_I2C_Mem_Write(&hi2c1, QMC5883L_ADDR, QMC5883L_REGISTER_3_ADDR, 1, &config_3_value, 1, 10);
  HAL_I2C_Mem_Write(&hi2c1, QMC5883L_ADDR, QMC5883L_REGISTER_1_ADDR, 1, &config_1_value, 1, 10);
}

void QMC5883L_SetStandbyMode() {
    uint8_t config_1_value = 0x00; // Che do StandbyMode
    HAL_I2C_Mem_Write(&hi2c1, QMC5883L_ADDR, QMC5883L_REGISTER_1_ADDR, 1, &config_1_value, 1, 10);
}

void QMC5883L_ReadData(int16_t* x, int16_t* y, int16_t* z) {
    uint8_t buff[6];
    HAL_I2C_Mem_Read(&hi2c1, QMC5883L_ADDR, 0x00, 1, buff, 6, 10);
    
    *x = (int16_t)((buff[1] << 8) | buff[0]);
    *y = (int16_t)((buff[3] << 8) | buff[2]);
    *z = (int16_t)((buff[5] << 8) | buff[4]);
}

void Caculate_AzimuthAngle(float* azimuth_angle, int16_t x, int16_t y){
	*azimuth_angle = atan2((float)y, (float)x);
	*azimuth_angle = *azimuth_angle * 180 / 3.1416;
	float declination_angle = 11.41666666666667;  
	*azimuth_angle += declination_angle;
		if(*azimuth_angle < 0){
			*azimuth_angle += 360;
		}
}

void LCD_Init(void) {
    HAL_Delay(50); 

    LCD_SendCommand(0x30); 
    HAL_Delay(5);
    LCD_SendCommand(0x30); 
    HAL_Delay(1);
    LCD_SendCommand(0x30); 
    HAL_Delay(10);
    LCD_SendCommand(0x20); 
    HAL_Delay(10);

    LCD_SendCommand(0x28); 
    HAL_Delay(1);
    LCD_SendCommand(0x08); 
    HAL_Delay(1);
    LCD_SendCommand(0x01); 
    HAL_Delay(2);
    LCD_SendCommand(0x06); 
    HAL_Delay(1);
    LCD_SendCommand(0x0C); 
    HAL_Delay(1);
}


void LCD_SendCommand(uint8_t cmd) {
    uint8_t data_u, data_l;
    uint8_t data_t[4];
    data_u = (cmd & 0xF0);
    data_l = ((cmd << 4) & 0xF0);
    data_t[0] = data_u | 0x0C; 
    data_t[1] = data_u | 0x08; 
    data_t[2] = data_l | 0x0C; 
    data_t[3] = data_l | 0x08; 
    HAL_I2C_Master_Transmit(&hi2c2, LCD_I2C_ADDRESS, (uint8_t *)data_t, 4, 100);
}

void LCD_SendData(uint8_t data) {
    uint8_t data_u, data_l;
    uint8_t data_t[4];
    data_u = (data & 0xF0);
    data_l = ((data << 4) & 0xF0);
    data_t[0] = data_u | 0x0D; 
    data_t[1] = data_u | 0x09; 
    data_t[2] = data_l | 0x0D; 
    data_t[3] = data_l | 0x09; 
    HAL_I2C_Master_Transmit(&hi2c2, LCD_I2C_ADDRESS, (uint8_t *)data_t, 4, 100);
}

void LCD_SendString(char *str, uint8_t row, uint8_t col) {
	LCD_SetCursor(row, col);
    while(*str != '\0') {
	LCD_SendData(*str);
	str++;
    }
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t address;

    switch (row) {
        case 0:
            address = 0x80 + col;
            break;
        case 1:
            address = 0xC0 + col;
            break;
        default:
            address = 0x80 + col;
            break;
    }

    LCD_SendCommand(address);
}

void LCD_ClearDisplay(){
	LCD_SendCommand(0x01);
	HAL_Delay(2);
}

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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	
  LCD_Init();
  LCD_ClearDisplay();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  htim1.Init.Prescaler = 15999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 199;
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
  htim2.Init.Prescaler = 15999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RED_LED_Pin|BLUE_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SW1_Pin SW2_Pin */
  GPIO_InitStruct.Pin = SW1_Pin|SW2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RED_LED_Pin BLUE_LED_Pin */
  GPIO_InitStruct.Pin = RED_LED_Pin|BLUE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Buzzer_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Buzzer_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
