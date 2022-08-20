/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include <string.h>
#include "MPU6050.h"
#include "ssd1306.h"
#include "ssd1306_defines.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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
DMA_HandleTypeDef hdma_i2c2_tx;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
RawData_Def myAccelRaw, myGyroRaw;
ScaledData_Def myAccelScaled, myGyroScaled;
int buff_size = 200;
float acc_queue[200][3] = { { 0 } };
float gyro_queue[200][3] = { { 0 } };
int acc_gyro_packet[6] = { 0 };

int count = 0;
int buff_read_count = 0;
int buff_write_count = 0;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	acc_gyro_packet[0] = acc_queue[buff_write_count][0];
	acc_gyro_packet[1] = acc_queue[buff_write_count][1];
	acc_gyro_packet[2] = acc_queue[buff_write_count][2];

	acc_gyro_packet[3] = gyro_queue[buff_write_count][0];
	acc_gyro_packet[4] = gyro_queue[buff_write_count][1];
	acc_gyro_packet[5] = gyro_queue[buff_write_count][2];

	HAL_UART_Transmit_IT(&huart1, acc_gyro_packet, sizeof(acc_gyro_packet));
	buff_write_count += 1;
	buff_write_count %= buff_size;
	count = 0;
}

void drawLines() {
	for (int16_t i = 0; i < ssd1306_GetWidth(); i += 4) {
		ssd1306_DrawLine(0, 0, i, ssd1306_GetHeight() - 1);
		ssd1306_UpdateScreen();
//		HAL_Delay(10);
	}
	for (int16_t i = 0; i < ssd1306_GetHeight(); i += 4) {
		ssd1306_DrawLine(0, 0, ssd1306_GetWidth() - 1, i);
		ssd1306_UpdateScreen();
//		HAL_Delay(10);
	}
//	HAL_Delay(250);

	ssd1306_Clear();
	for (int16_t i = 0; i < ssd1306_GetWidth(); i += 4) {
		ssd1306_DrawLine(0, ssd1306_GetHeight() - 1, i, 0);
		ssd1306_UpdateScreen();
//		HAL_Delay(10);
	}
	for (int16_t i = ssd1306_GetHeight() - 1; i >= 0; i -= 4) {
		ssd1306_DrawLine(0, ssd1306_GetHeight() - 1, ssd1306_GetWidth() - 1, i);
		ssd1306_UpdateScreen();
//		HAL_Delay(10);
	}
//	HAL_Delay(250);
	ssd1306_Clear();
	for (int16_t i = ssd1306_GetWidth() - 1; i >= 0; i -= 4) {
		ssd1306_DrawLine(ssd1306_GetWidth() - 1, ssd1306_GetHeight() - 1, i, 0);
		ssd1306_UpdateScreen();
//		HAL_Delay(10);
	}
	for (int16_t i = ssd1306_GetHeight() - 1; i >= 0; i -= 4) {
		ssd1306_DrawLine(ssd1306_GetWidth() - 1, ssd1306_GetHeight() - 1, 0, i);
		ssd1306_UpdateScreen();
//		HAL_Delay(10);
	}
//	HAL_Delay(250);
	ssd1306_Clear();
	for (int16_t i = 0; i < ssd1306_GetHeight(); i += 4) {
		ssd1306_DrawLine(ssd1306_GetWidth() - 1, 0, 0, i);
		ssd1306_UpdateScreen();
//		HAL_Delay(10);
	}
	for (int16_t i = 0; i < ssd1306_GetWidth(); i += 4) {
		ssd1306_DrawLine(ssd1306_GetWidth() - 1, 0, i, ssd1306_GetHeight() - 1);
		ssd1306_UpdateScreen();
//		HAL_Delay(10);
	}
//	HAL_Delay(250);
}

// Adapted from Adafruit_SSD1306
void drawRect(void) {
	for (int16_t i = 0; i < ssd1306_GetHeight() / 2; i += 2) {
		ssd1306_DrawRect(i, i, ssd1306_GetWidth() - 2 * i,
				ssd1306_GetHeight() - 2 * i);
		ssd1306_UpdateScreen();
//		HAL_Delay(10);
	}
}

// Adapted from Adafruit_SSD1306
void fillRect(void) {
	uint8_t color = 1;
	for (int16_t i = 0; i < ssd1306_GetHeight() / 2; i += 3) {
		ssd1306_SetColor((color % 2 == 0) ? Black : White); // alternate colors
		ssd1306_FillRect(i, i, ssd1306_GetWidth() - i * 2,
				ssd1306_GetHeight() - i * 2);
		ssd1306_UpdateScreen();
//		HAL_Delay(10);
		color++;
	}
	// Reset back to WHITE
	ssd1306_SetColor(White);
}

// Adapted from Adafruit_SSD1306
void drawCircle(void) {
	for (int16_t i = 0; i < ssd1306_GetHeight(); i += 2) {
		ssd1306_DrawCircle(ssd1306_GetWidth() / 2, ssd1306_GetHeight() / 2, i);
		ssd1306_UpdateScreen();
		HAL_Delay(10);
	}
	HAL_Delay(1000);
	ssd1306_Clear();

	// This will draw the part of the circel in quadrant 1
	// Quadrants are numberd like this:
	//   0010 | 0001
	//  ------|-----
	//   0100 | 1000
	//
	ssd1306_DrawCircleQuads(ssd1306_GetWidth() / 2, ssd1306_GetHeight() / 2,
			ssd1306_GetHeight() / 4, 0x01 /*0b00000001*/);
	ssd1306_UpdateScreen();
//	HAL_Delay(200);
	ssd1306_DrawCircleQuads(ssd1306_GetWidth() / 2, ssd1306_GetHeight() / 2,
			ssd1306_GetHeight() / 4, 0x03 /*0b00000011*/);
	ssd1306_UpdateScreen();
//	HAL_Delay(200);
	ssd1306_DrawCircleQuads(ssd1306_GetWidth() / 2, ssd1306_GetHeight() / 2,
			ssd1306_GetHeight() / 4, 0x07 /*0b00000111*/);
	ssd1306_UpdateScreen();
//	HAL_Delay(200);
	ssd1306_DrawCircleQuads(ssd1306_GetWidth() / 2, ssd1306_GetHeight() / 2,
			ssd1306_GetHeight() / 4, 0x0F /*0b00001111*/);
	ssd1306_UpdateScreen();
}

void drawProgressBarDemo(int counter) {
	char str[128];
	// draw the progress bar
	ssd1306_DrawProgressBar(0, 32, 120, 10, counter);

	// draw the percentage as String
	ssd1306_SetCursor(64, 15);
	sprintf(str, "%i%%", counter);
	ssd1306_WriteString(str, Font_7x10);
	ssd1306_UpdateScreen();
}

void mainApp(void) {
	drawLines();
	ssd1306_Clear();

	drawRect();
	ssd1306_Clear();

	fillRect();
	ssd1306_Clear();

	drawCircle();
	ssd1306_Clear();

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	MPU_ConfigTypeDef myMpuConfig;
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

	MPU6050_Init(&hi2c1);
	ssd1306_Init();
	ssd1306_FlipScreenVertically();
	ssd1306_Clear();
	ssd1306_SetColor(White);

	//2. Configure Accel and Gyro parameters
	myMpuConfig.Accel_Full_Scale = AFS_SEL_4g;
	myMpuConfig.ClockSource = Internal_8MHz;
	myMpuConfig.CONFIG_DLPF = DLPF_184A_188G_Hz;
	myMpuConfig.Gyro_Full_Scale = FS_SEL_500;
	myMpuConfig.Sleep_Mode_Bit = 0;  //1: sleep mode, 0: normal mode
	MPU6050_Config(&myMpuConfig);

	HAL_UART_Transmit_IT(&huart1, acc_gyro_packet, sizeof(acc_gyro_packet));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1) {
		MPU6050_Get_Accel_Scale(&myAccelScaled);
		MPU6050_Get_Gyro_Scale(&myGyroScaled);

		acc_queue[buff_read_count % buff_size][0] = myAccelScaled.x;
		acc_queue[buff_read_count % buff_size][1] = myAccelScaled.y;
		acc_queue[buff_read_count % buff_size][2] = myAccelScaled.z;

		gyro_queue[buff_read_count % buff_size][0] = myGyroScaled.x;
		gyro_queue[buff_read_count % buff_size][1] = myGyroScaled.y;
		gyro_queue[buff_read_count % buff_size][2] = myGyroScaled.z;

		buff_read_count += 1;
		buff_read_count %= buff_size;
		count++;
		HAL_Delay(30);
		mainApp();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

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
  hi2c1.Init.ClockSpeed = 400000;
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
  hi2c2.Init.ClockSpeed = 400000;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
	while (1) {
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
