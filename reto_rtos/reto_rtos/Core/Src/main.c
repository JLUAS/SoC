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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include "EngTrModel.h"
#include "rtwtypes.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// LCD Display
#define RS_Pin GPIO_PIN_4
#define RS_GPIO_Port GPIOC
#define E_Pin GPIO_PIN_0
#define E_GPIO_Port GPIOA
#define D4_Pin GPIO_PIN_1
#define D4_GPIO_Port GPIOA
#define D5_Pin GPIO_PIN_4
#define D5_GPIO_Port GPIOA
#define D6_Pin GPIO_PIN_0
#define D6_GPIO_Port GPIOB
#define D7_Pin GPIO_PIN_1
#define D7_GPIO_Port GPIOC

// Matrix Keypad
#define mpo_0_PORT GPIOA
#define mpo_0_PIN  GPIO_PIN_5  // D13
#define mpo_1_PORT GPIOA
#define mpo_1_PIN  GPIO_PIN_6  // D12
#define mpo_2_PORT GPIOA
#define mpo_2_PIN  GPIO_PIN_7  // D11
#define mpo_3_PORT GPIOB
#define mpo_3_PIN  GPIO_PIN_6  // D10
#define mpi_0_PORT GPIOC
#define mpi_0_PIN  GPIO_PIN_7  // D9
#define mpi_1_PORT GPIOA
#define mpi_1_PIN  GPIO_PIN_9  // D8
#define mpi_2_PORT GPIOA
#define mpi_2_PIN  GPIO_PIN_8  // D7
#define mpi_3_PORT GPIOB
#define mpi_3_PIN  GPIO_PIN_10 // D6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId LeerValoresHandle;
osThreadId EjecutarModeloHandle;
osThreadId EnviarUARTHandle;
osThreadId EnviarLCDHandle;
osMessageQId keyHandle;
osMessageQId valueHandle;
osMessageQId throttleHandle;
osMessageQId brake_torqueHandle;
osMessageQId steeringHandle;
osMessageQId speedHandle;
osMessageQId engineHandle;
osMessageQId gearHandle;
osMutexId taskMutexHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void const * argument);
void StartLeerValores(void const * argument);
void StartEjecutarModelo(void const * argument);
void StartEnviarUART(void const * argument);
void StartEnviarLCD(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Otras
void FormatFloat(char *buffer, float value, bool fixed_decimals) {
    int int_part = (int)value;                          // Extract integer part
    int decimal_part = (int)((value - int_part) * 100); // Extract two decimal places

    if (fixed_decimals) {
        // Always display two decimals
        sprintf(buffer, "%d.%02d", int_part, abs(decimal_part));
    } else {
        // Dynamically hide trailing zeros
        if (decimal_part == 0) {
            sprintf(buffer, "%d", int_part); // No decimals
        } else if (decimal_part % 10 == 0) {
            sprintf(buffer, "%d.%d", int_part, abs(decimal_part / 10)); // One decimal place
        } else {
            sprintf(buffer, "%d.%02d", int_part, abs(decimal_part)); // Two decimal places
        }
    }
}

// FUNCIONES RTOS
uint32_t GetAndReplaceValue(osMessageQId queueHandle) {
    uint32_t value = 0;  // Local variable to hold the value
    osEvent r_event = osMessageGet(queueHandle, 0);

    if (r_event.status == osEventMessage) {
        value = r_event.value.v;  // Store the value from the queue
        osMessagePut(queueHandle, value, 0);  // Immediately replace the value in the queue
    }

    return value;  // Return the value
}
void PutValueInQueue(osMessageQId queueHandle, void *value) {
    // Try to put the value in the queue
    osStatus status = osMessagePut(queueHandle, (uint32_t)value, 0);
    if (status != osOK) {
        // If the queue is full, remove the oldest value and try again
        osMessageGet(queueHandle, 0);  // Remove the oldest value
        osMessagePut(queueHandle, (uint32_t)value, 0);  // Add the new value
    }
}
void DebugTaskTiming(char task_id, uint32_t time) {
    time = time % 1000;  // Keep the output within 3 digits
    char debug_msg[8];   // Buffer for message (7 characters + null terminator)
    sprintf(debug_msg, "%c%03lu\r\n", task_id, time);  // Format message using %lu for uint32_t
    HAL_UART_Transmit(&huart2, (uint8_t *)debug_msg, strlen(debug_msg), HAL_MAX_DELAY);
}
int _write(int file, char *ptr, int len){
	int DataIdx;
	for(DataIdx=0; DataIdx<len; DataIdx++){
	while(!( USART2->SR & USART_SR_TXE ));
		USART2->DR = *ptr++;
	}
	return len;
}

// FUNCIONES KEYPAD
int8_t read_matrix_inputs(void) {
    // Read the state of each defined input pin
    uint8_t mpi_0_state = HAL_GPIO_ReadPin(mpi_0_PORT, mpi_0_PIN);
    uint8_t mpi_1_state = HAL_GPIO_ReadPin(mpi_1_PORT, mpi_1_PIN);
    uint8_t mpi_2_state = HAL_GPIO_ReadPin(mpi_2_PORT, mpi_2_PIN);
    uint8_t mpi_3_state = HAL_GPIO_ReadPin(mpi_3_PORT, mpi_3_PIN);

    // Return a number based on which input is on
    if (mpi_0_state == GPIO_PIN_SET) {
        return 0;
    } else if (mpi_1_state == GPIO_PIN_SET) {
        return 1;
    } else if (mpi_2_state == GPIO_PIN_SET) {
        return 2;
    } else if (mpi_3_state == GPIO_PIN_SET) {
        return 3;
    }

    // If no input is on, return -1 (or another invalid value to indicate no input)
    return -1;
}
int8_t read_matrix_pad(void) {
    int32_t col = -1;
    int32_t key = -1;

    // Set mpo_0 (pin index 0) to 1
    HAL_GPIO_WritePin(mpo_0_PORT, mpo_0_PIN, GPIO_PIN_SET);
    col = read_matrix_inputs();
    if (col != -1) {
        key = 0 * 4 + col;
    }
    HAL_GPIO_WritePin(mpo_0_PORT, mpo_0_PIN, GPIO_PIN_RESET);  // Reset after check

    // Set mpo_1 (pin index 1) to 1
    HAL_GPIO_WritePin(mpo_1_PORT, mpo_1_PIN, GPIO_PIN_SET);
    col = read_matrix_inputs();
    if (col != -1) {
        key = 1 * 4 + col;
    }
    HAL_GPIO_WritePin(mpo_1_PORT, mpo_1_PIN, GPIO_PIN_RESET);  // Reset after check

    // Set mpo_2 (pin index 2) to 1
    HAL_GPIO_WritePin(mpo_2_PORT, mpo_2_PIN, GPIO_PIN_SET);
    col = read_matrix_inputs();
    if (col != -1) {
        key = 2 * 4 + col;
    }
    HAL_GPIO_WritePin(mpo_2_PORT, mpo_2_PIN, GPIO_PIN_RESET);  // Reset after check

    // Set mpo_3 (pin index 3) to 1
    HAL_GPIO_WritePin(mpo_3_PORT, mpo_3_PIN, GPIO_PIN_SET);
    col = read_matrix_inputs();
    if (col != -1) {
        key = 3 * 4 + col;
    }
    HAL_GPIO_WritePin(mpo_3_PORT, mpo_3_PIN, GPIO_PIN_RESET);  // Reset after check

    // If no valid col found, return 255 (or any other invalid marker)
    return key;
}

// FUNCIONES LCD
void LCD_Command(uint8_t cmd) {
    // Send high nibble
    HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_RESET); // RS = 0 for command
    HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, (cmd >> 4) & 0x01);
    HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, (cmd >> 5) & 0x01);
    HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, (cmd >> 6) & 0x01);
    HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, (cmd >> 7) & 0x01);

    HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_SET);  // Enable pin HIGH
    HAL_Delay(0.00001);
    HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_RESET);  // Enable pin LOW
    HAL_Delay(0.00001);

    // Send low nibble
    HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, cmd & 0x01);
    HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, (cmd >> 1) & 0x01);
    HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, (cmd >> 2) & 0x01);
    HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, (cmd >> 3) & 0x01);

    HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_SET);  // Enable pin HIGH
    HAL_Delay(0.00001);
    HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_RESET);  // Enable pin LOW
    HAL_Delay(0.00001);
}
void LCD_Write_Char(char data) {
    // Send high nibble
    HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_SET); // RS = 1 for data
    HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, (data >> 4) & 0x01);
    HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, (data >> 5) & 0x01);
    HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, (data >> 6) & 0x01);
    HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, (data >> 7) & 0x01);

    HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_SET);  // Enable pin HIGH
    HAL_Delay(0.00001);
    HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_RESET);  // Enable pin LOW
    HAL_Delay(0.00001);

    // Send low nibble
    HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, data & 0x01);
    HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, (data >> 1) & 0x01);
    HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, (data >> 2) & 0x01);
    HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, (data >> 3) & 0x01);

    HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_SET);  // Enable pin HIGH
    HAL_Delay(0.00001);
    HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_RESET);  // Enable pin LOW
    HAL_Delay(0.00001);
}
void LCD_Init(void) {
    HAL_Delay(20);  // Wait for 20ms
    LCD_Command(0x03);
    HAL_Delay(5);
    LCD_Command(0x03);
    HAL_Delay(1);
    LCD_Command(0x03);
    LCD_Command(0x02);  // 4-bit mode
    LCD_Command(0x28);  // 2 lines, 5x7 matrix
    LCD_Command(0x0C);  // Display on, cursor off
    LCD_Command(0x06);  // Increment cursor
    LCD_Command(0x01);  // Clear display
    HAL_Delay(2);
}
void LCD_Set_Cursor(uint8_t row, uint8_t col) {
    uint8_t pos;
    if (row == 0) {
        pos = 0x80 + col;
    } else {
        pos = 0xC0 + col;
    }
    LCD_Command(pos);
}
void LCD_Write_String(char *str) {
    while (*str) {
        LCD_Write_Char(*str++);
    }
}
void LCD_Overwrite(char *str) {
	LCD_Command(0x01);  	// Clear
	HAL_Delay(1);	  		// Wait for clear
	LCD_Set_Cursor(0, 0);	// Reset cursor
    while (*str) {
        if (*str == '\n') {
            LCD_Set_Cursor(1, 0);  // Move to the beginning of the second line
        } else {
            LCD_Write_Char(*str);
        }
        str++;
    }
}
void LCD_Update(char *str) {
    static char prev_str[32] = "";  // Assume 16x2 LCD, so max 32 characters
    char padded_str[32] = "";       // Buffer for the padded version of `str`
    size_t len = strlen(str);

    // Pad the input string to match the LCD's dimensions
    memset(padded_str, ' ', sizeof(padded_str));  // Fill with spaces
    if (len > 0) {
        // Copy the input string into the padded buffer
        for (size_t i = 0, lcd_pos = 0; i < len && lcd_pos < 32; i++) {
            if (str[i] == '\n') {
                // Move to the second line at index 16
                lcd_pos = 16;
            } else if (lcd_pos < 32) {
                // Copy the character to the padded buffer
                padded_str[lcd_pos++] = str[i];
            }
        }
    }

    // Compare each character with the previous display and update if needed
    for (size_t i = 0; i < 32; i++) {
        if (padded_str[i] != prev_str[i]) {
            LCD_Set_Cursor(i / 16, i % 16);  // Determine row and column
            LCD_Write_Char(padded_str[i]);  // Write the updated character
            prev_str[i] = padded_str[i];    // Update the prev_str record
        }
    }
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of taskMutex */
  osMutexDef(taskMutex);
  taskMutexHandle = osMutexCreate(osMutex(taskMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of key */
  osMessageQDef(key, 1, uint32_t);
  keyHandle = osMessageCreate(osMessageQ(key), NULL);

  /* definition and creation of value */
  osMessageQDef(value, 1, uint32_t);
  valueHandle = osMessageCreate(osMessageQ(value), NULL);

  /* definition and creation of throttle */
  osMessageQDef(throttle, 1, uint32_t);
  throttleHandle = osMessageCreate(osMessageQ(throttle), NULL);

  /* definition and creation of brake_torque */
  osMessageQDef(brake_torque, 1, uint32_t);
  brake_torqueHandle = osMessageCreate(osMessageQ(brake_torque), NULL);

  /* definition and creation of steering */
  osMessageQDef(steering, 1, uint32_t);
  steeringHandle = osMessageCreate(osMessageQ(steering), NULL);

  /* definition and creation of speed */
  osMessageQDef(speed, 1, uint32_t);
  speedHandle = osMessageCreate(osMessageQ(speed), NULL);

  /* definition and creation of engine */
  osMessageQDef(engine, 1, uint32_t);
  engineHandle = osMessageCreate(osMessageQ(engine), NULL);

  /* definition and creation of gear */
  osMessageQDef(gear, 1, uint32_t);
  gearHandle = osMessageCreate(osMessageQ(gear), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of LeerValores */
  osThreadDef(LeerValores, StartLeerValores, osPriorityNormal, 0, 256);
  LeerValoresHandle = osThreadCreate(osThread(LeerValores), NULL);

  /* definition and creation of EjecutarModelo */
  osThreadDef(EjecutarModelo, StartEjecutarModelo, osPriorityNormal, 0, 256);
  EjecutarModeloHandle = osThreadCreate(osThread(EjecutarModelo), NULL);

  /* definition and creation of EnviarUART */
  osThreadDef(EnviarUART, StartEnviarUART, osPriorityNormal, 0, 256);
  EnviarUARTHandle = osThreadCreate(osThread(EnviarUART), NULL);

  /* definition and creation of EnviarLCD */
  osThreadDef(EnviarLCD, StartEnviarLCD, osPriorityNormal, 0, 256);
  EnviarLCDHandle = osThreadCreate(osThread(EnviarLCD), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 PA5
                           PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	// Calendarización para esta tarea
	const int period = 100;
	const int offset = 5;
	osDelay(offset);

  /* Infinite loop */
	for(;;)
	{
		//int tick = osKernelSysTick();
		//printf("Hello RTOS World!! Tick: %d\r\n", tick);

		// Random delay to simulate longer execution
		//int random_delay = 10 + rand() % 100;
		//HAL_Delay(random_delay);

		// Wait until next release time
		//int cur_tick = osKernelSysTick();
		//osDelay(((cur_tick / period) + 1) * period + offset - cur_tick);
		osDelay(1);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartLeerValores */
/**
* @brief Function implementing the LeerValores thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLeerValores */
void StartLeerValores(void const * argument)
{
  /* USER CODE BEGIN StartLeerValores */
	// Calendarización para esta tarea
	const int period = 40;
	const int offset = 0;
	osDelay(offset);

	uint16_t mini = 500;	// Mínimo valor del ADC
	uint16_t maxi = 4000;	// Máximo valor del ADC
	int8_t key = -1;
	uint16_t value = 0;
	float brake_torque = 0;
	float throttle;
	int8_t steering = 0;
	char msg[256] = "";

	/* Infinite loop */
	for(;;)
	{
	  	// Start
		osMutexWait(taskMutexHandle, osWaitForever);
		uint32_t start_tick = osKernelSysTick();  // Get start time
		//DebugTaskTiming('L', osKernelSysTick()); // Ver inicio

		// Leemos el teclado
		key = read_matrix_pad();
		PutValueInQueue(keyHandle, &key);  // Use the new function to handle key

		// Leemos potenciometro
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 20);
		value = HAL_ADC_GetValue(&hadc1);
		PutValueInQueue(valueHandle, &value);  // Use the new function to handle value

		// Calculamos Throttle
		throttle = fmax(0.0f, fmin(((float)value - (float)mini) / ((float)maxi - (float)mini), 1.0f));
		PutValueInQueue(throttleHandle, &throttle);  // Use the new function to handle throttle

		// Calculamos brake torque
		brake_torque = (key == 5) ? 1 : 0;
		PutValueInQueue(brake_torqueHandle, &brake_torque);  // Use the new function to handle brake_torque

		// Calculamos dirección
		if (key == 4)
		    steering = -1;
		else if (key == 6)
		    steering = 1;
		else
		    steering = 0;
		PutValueInQueue(steeringHandle, &steering);  // Use the new function to handle steering

		// Wait until next release time
		osMutexRelease(taskMutexHandle); // Done
		//DebugTaskTiming('L', osKernelSysTick() - start_tick); //  Ver duración
		int cur_tick = osKernelSysTick();
		osDelay(((cur_tick / period) + 1) * period + offset - cur_tick);
	}
  /* USER CODE END StartLeerValores */
}

/* USER CODE BEGIN Header_StartEjecutarModelo */
/**
* @brief Function implementing the EjecutarModelo thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEjecutarModelo */
void StartEjecutarModelo(void const * argument)
{
  /* USER CODE BEGIN StartEjecutarModelo */
	  // Calendarización para esta tarea
	  const int period = 40;
	  const int offset = 6;
	  osDelay(offset);

	  osEvent r_event;
	  bool stopped = false;
      float mint = 1.45;
	  float maxt = 100;
      float max_bt = 2000;
	  float brake_torque = 0;
	  float throttle = 0;
	  float speed = 0.0;
	  float engine = 0.0;
	  int8_t gear = 0;
	  /* Infinite loop */
	  for(;;)
	  {
		  	// Start
		    osMutexWait(taskMutexHandle, osWaitForever);
		  	uint32_t start_tick = osKernelSysTick();  // Get start time
			//DebugTaskTiming('M', osKernelSysTick()); // Ver inicio

			// Obtenemos throttle y brake_torque
			throttle = *(float *)GetAndReplaceValue(throttleHandle);
			throttle = throttle * (maxt - mint) + mint;
			brake_torque = *(float *)GetAndReplaceValue(brake_torqueHandle);
			brake_torque = brake_torque * max_bt;

			// Asignamos throttle y brake_torque
			EngTrModel_U.Throttle = throttle; // Get
			EngTrModel_U.BrakeTorque = brake_torque; // Get

			// Ejecutamos el modelo
			EngTrModel_step();

			// Checamos que la velocidad no sea negativa
			if (EngTrModel_Y.VehicleSpeed < 0)
				EngTrModel_Y.VehicleSpeed = 0;

			// Apply the stopped condition
			if (stopped)
			    EngTrModel_Y.VehicleSpeed = 0;  // Force speed to remain 0 while stopped

			// Update the stopped flag
			stopped = (EngTrModel_Y.VehicleSpeed == 0 && brake_torque > 0);

			// Leemos salida del modelo
			speed = EngTrModel_Y.VehicleSpeed;	// Put
			engine = EngTrModel_Y.EngineSpeed;	// Put
			gear = EngTrModel_Y.Gear;			// Put

			// Enviamos a queues
			PutValueInQueue(speedHandle, &speed);   // Use the new function to handle speed
			PutValueInQueue(engineHandle, &engine); // Use the new function to handle engine
			PutValueInQueue(gearHandle, &gear);     // Use the new function to handle gear

			// Wait until next release time
			osMutexRelease(taskMutexHandle); // Done
			//DebugTaskTiming('M', osKernelSysTick() - start_tick); // Ver duración
			int cur_tick = osKernelSysTick();
			osDelay(((cur_tick / period) + 1) * period + offset - cur_tick);
	  }
  /* USER CODE END StartEjecutarModelo */
}

/* USER CODE BEGIN Header_StartEnviarUART */
/**
* @brief Function implementing the EnviarUART thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEnviarUART */
void StartEnviarUART(void const * argument)
{
  /* USER CODE BEGIN StartEnviarUART */
	  // Calendarización para esta tarea
	  const int period = 40;
	  const int offset = 10;
	  osDelay(offset);

	  osEvent r_event;
	  uint8_t msg[256] = "";
	  int8_t key = -1;			// Queue
	  uint16_t value = 0;		// Queue
	  int8_t steering = 0;		// Queue
	  int8_t gear = 0;			// Queue
	  float throttle;			// Queue
	  float speed = 0.0;		// Queue
	  float engine = 0.0;		// Queue
	  float brake_torque = 0;	// Queue

	  bool transmit = false;
	  /* Infinite loop */
	  for(;;)
	  {
		  	// Start
			osMutexWait(taskMutexHandle, osWaitForever);
			uint32_t start_tick = osKernelSysTick();  // Get start time
			//DebugTaskTiming('U', osKernelSysTick()); // Ver inicio

			// Get values
			key = *(int8_t *)GetAndReplaceValue(keyHandle);
			value = *(uint16_t *)GetAndReplaceValue(valueHandle);
			steering = *(int8_t *)GetAndReplaceValue(steeringHandle);
			gear = *(int8_t *)GetAndReplaceValue(gearHandle);
			throttle = *(float *)GetAndReplaceValue(throttleHandle);
			speed = *(float *)GetAndReplaceValue(speedHandle);
			engine = *(float *)GetAndReplaceValue(engineHandle);
			brake_torque = *(float *)GetAndReplaceValue(brake_torqueHandle);

			// Check if key 12 or 14 is pressed
			if (key == 12)
				transmit = false;
			else if (key == 14)
				transmit = true;

			// Main sprintf logic for the message
			char throttle_str[10];
			FormatFloat(throttle_str, throttle, false);   // Format throttle
			sprintf(msg, "Key: %d\tADC: %u\tThr: %s\tBrk: %u\tStr: %d\tG: %u\tEng: %d.%02d\tSpd: %d.%02d\t%s\r\n",
			        key,
			        value,
			        throttle_str,
			        (int)brake_torque,
			        steering,
			        gear,
			        (int)speed, (int)((speed - (int)speed) * 100),
			        (int)engine, (int)((engine - (int)engine) * 100),
			        transmit ? "\tT" : "\tX");
			printf(msg);

			// Wait until next release time
			osMutexRelease(taskMutexHandle); // Done
			//DebugTaskTiming('U', osKernelSysTick() - start_tick); // Ver duración
			int cur_tick = osKernelSysTick();
			osDelay(((cur_tick / period) + 1) * period + offset - cur_tick);
	  }
  /* USER CODE END StartEnviarUART */
}

/* USER CODE BEGIN Header_StartEnviarLCD */
/**
* @brief Function implementing the EnviarLCD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEnviarLCD */
void StartEnviarLCD(void const * argument)
{
  /* USER CODE BEGIN StartEnviarLCD */
	  LCD_Init();
	  // Calendarización para esta tarea
	  const int period = 120;
	  const int offset = 17;
	  const int cycle_duration = 5000;
	  osDelay(offset);

	  osEvent r_event;
	  char lcd_msg[33] = "";
	  int8_t key = -1, prev_key = -1;  // Current and previous key state
	  int display_counter = 0;        // Counter to manually cycle through displays
	  const int num_displays = 4;     // Total number of displays

	  uint16_t value = 0;
	  float brake_torque = 0;
	  float throttle = 0;
	  int8_t steering = 0;
	  int8_t gear = 0;
	  float speed = 0.0;
	  float engine = 0.0;

	  /* Infinite loop */
	  for(;;)
	  {
		// Start
		osMutexWait(taskMutexHandle, osWaitForever);
		uint32_t start_tick = osKernelSysTick();  // Get start time
		//DebugTaskTiming('P', osKernelSysTick()); // Ver inicio

		// Get values
		key = *(int8_t *)GetAndReplaceValue(keyHandle);
		value = *(uint16_t *)GetAndReplaceValue(valueHandle);
		brake_torque = *(float *)GetAndReplaceValue(brake_torqueHandle);
		throttle = *(float *)GetAndReplaceValue(throttleHandle);
		steering = *(int8_t *)GetAndReplaceValue(steeringHandle);
		gear = *(int8_t *)GetAndReplaceValue(gearHandle);
		speed = *(float *)GetAndReplaceValue(speedHandle);
		engine = *(float *)GetAndReplaceValue(engineHandle);

		// Check for key press event (key changes from unpressed to pressed)
		if (key == 13 && prev_key != 13)
			display_counter = (display_counter + 1) % num_displays;  // Cycle display manually
		prev_key = key;  // Update previous key state

		// Determine the active display
		int active_display;
		if (display_counter == 0) {
			// Automatic cycling mode
			int tick = osKernelSysTick();
			active_display = 1 + ((tick / cycle_duration) % (num_displays - 1));  // Alternates between 1 and 2
		} else {
			// Manual selection mode
			active_display = display_counter;
		}

		if (active_display == 1) {
		    // Display 1: Outputs del modelo
		    sprintf(lcd_msg, "G:%d Eng:%d\nSpd:%d", gear, (int)engine, (int)speed);
		} else if (active_display == 2) {
			// Display 2: Inputs del modelo
		    char throttle_str[10], brake_torque_str[10];
		    FormatFloat(throttle_str, throttle, true);         // Format throttle with fixed decimals

		    sprintf(lcd_msg, "T:%s B:%d\nS:%d", throttle_str, (int)brake_torque, (int)steering);
		} else if (active_display == 3) {
		    // Display 3: Inputs del sistema
		    sprintf(lcd_msg, "Key:%d\nADC:%u",
		            key,                                // Key as integer
		            value);                             // ADC value as unsigned integer
		}

		LCD_Update(lcd_msg);

		// Wait until next release time
		osMutexRelease(taskMutexHandle); // Done
		//DebugTaskTiming('P', osKernelSysTick() - start_tick); // Ver duración
		uint32_t cur_tick = osKernelSysTick();
		osDelay(((cur_tick / period) + 1) * period + offset - cur_tick);
	  }
  /* USER CODE END StartEnviarLCD */
}



/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
