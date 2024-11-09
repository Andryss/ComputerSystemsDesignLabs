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
#include "buf.h"
#include "btn.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ST_MAIN             0  /* main work mode */
#define ST_EDIT_WAIT_MODE   1  /* edit mode, wait for mode selection */
#define ST_EDIT_WAIT_LED    2  /* edit mode, wait for led selection */
#define ST_EDIT_WAIT_BRIGHT 3  /* edit mode, wait for led brightness selection */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
struct Buffer receiveBuffer = {0};
struct Buffer transmitBuffer = {0};

uint8_t char_buf[1] = {'\0'};

void transmit(uint8_t* data, size_t size) {
	if (HAL_UART_Transmit_IT(&huart6, data, size) == HAL_BUSY) {
		buf_push(&transmitBuffer, data, size);
	}
}

void transmitc(char* data) {
	transmit((uint8_t*) data, strlen(data));
}

void receive() {
	HAL_UART_Receive_IT(&huart6, char_buf, sizeof(char_buf));
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	buf_push(&receiveBuffer, char_buf, sizeof(char_buf));
	transmit(char_buf, sizeof(char_buf));
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  uint8_t buf[1024];
  if (buf_pop(&transmitBuffer, buf, sizeof(buf))) {
	  HAL_UART_Transmit_IT(&huart6, buf, strlen((char*) buf));
  }
}

bool hello_print = false;
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
  buf_init(&receiveBuffer);
  buf_init(&transmitBuffer);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART6_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  uint16_t mode_leds[9] = {0x1, 0x1, 0x1, 0x2, 0x2, 0x2, 0x4, 0x4, 0x4};
  uint16_t mode_brightness[9] = {10, 40, 100, 10, 40, 100, 10, 40, 100};

  uint16_t state = ST_MAIN;

  size_t cur_mode = -1;

  size_t edit_mode = 0;
  uint16_t edit_mode_led = 0;
  uint16_t edit_mode_brightness = 0;

  void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	  if (htim->Instance == TIM6) {
		  if (cur_mode != -1) {
			  uint16_t leds = mode_leds[cur_mode];
			  uint16_t brightness = mode_brightness[cur_mode];

			  if ((leds & 0x1) != 0) {
				  htim4.Instance->CCR2 = brightness * 10;
			  } else {
				  htim4.Instance->CCR2 = 0;
			  }

			  if ((leds & 0x2) != 0) {
				  htim4.Instance->CCR3 = brightness * 10;
			  } else {
				  htim4.Instance->CCR3 = 0;
			  }

			  if ((leds & 0x4) != 0) {
				  htim4.Instance->CCR4 = brightness * 10;
			  } else {
				  htim4.Instance->CCR4 = 0;
			  }

			  cur_mode = -1;
		  }
	  }
  }

  char leds_str(uint16_t leds) {
	  if (leds == 0x1) return 'G';
	  if (leds == 0x2) return 'Y';
	  if (leds == 0x4) return 'R';
	  return '?';
  }

  void print_mode_message() {
	  char buf[256];

	  char mode_leds_str[9] = {0};
	  for (size_t i = 0; i < sizeof(mode_leds_str); i++) {
		  mode_leds_str[i] = leds_str(mode_leds[i]);
	  }

	  char *format = "\
			  Modes:\r\n\
			  1 - %s %d%%\r\n\
			  2 - %s %d%%\r\n\
			  3 - %s %d%%\r\n\
			  4 - %s %d%%\r\n\
			  5 - %s %d%%\r\n\
			  6 - %s %d%%\r\n\
			  7 - %s %d%%\r\n\
			  8 - %s %d%%\r\n\
			  9 - %s %d%%\r\n\
			  > ";

	  snprintf(buf, sizeof(buf), format,
			  mode_leds_str[0], mode_brightness[0], mode_leds_str[1], mode_brightness[1], mode_leds_str[2], mode_brightness[2],
			  mode_leds_str[3], mode_brightness[3], mode_leds_str[4], mode_brightness[4], mode_leds_str[5], mode_brightness[5],
			  mode_leds_str[6], mode_brightness[6], mode_leds_str[7], mode_brightness[7], mode_leds_str[8], mode_brightness[8]);

	  transmitc(buf);
  }

  void handle_command(char cmd) {
	  if (state == ST_MAIN) {
		  if (cmd >= '1' && cmd <= '9') {
			  cur_mode = (cmd - '1');
			  char buf[32];
			  snprintf(buf, sizeof(buf), "Mode %c selected\n\r> ", cmd);
			  transmitc(buf);
		  } else if (cmd == '0') {
			  cur_mode = -1;
			  transmitc("All LEDs off\n\r> ");
		  } else if (cmd == '\r') {
			  state = ST_EDIT_WAIT_MODE;
			  edit_mode = -1;
			  transmitc("Enter edit mode\r\nChoose mode to edit [1-9]: ");
		  } else {
			  transmitc("Unknown command (available: 1-9,0,Enter)\n\r> ");
		  }
	  } else if (state == ST_EDIT_WAIT_MODE) {
		  if (cmd >= '1' && cmd <= '9') {
			  edit_mode = (cmd - '1');
			  char buf[128];
			  snprintf(buf, sizeof(buf), "Selected mode %c. Enter to continue or choose another mode [1-9]: ", cmd);
			  transmitc(buf);
		  } else if (cmd == '\r') {
			  if (edit_mode == -1) {
				  transmitc("No mode selected. Choose mode to edit [1-9]: ");
			  } else {
				  state = ST_EDIT_WAIT_LED;
				  edit_mode_led = 0;
				  transmitc("Choose led for mode [g,y,r]: ");
			  }
		  } else {
			  if (edit_mode == -1) {
				  transmitc("Unknown command. Choose mode to edit [1-9]: ");
			  } else {
				  char buf[128];
				  snprintf(buf, sizeof(buf), "Unknown command. Selected mode %c. Enter to continue or choose another mode [1-9]: ", cmd);
				  transmitc(buf);
			  }
		  }
	  } else if (state == ST_EDIT_WAIT_LED) {
		  if (cmd == 'g') {
			  edit_mode_led = 0x1;
			  transmitc("Green led selected. Enter to continue or choose another led [g,y,r]: ");
		  } else if (cmd == 'y') {
			  edit_mode_led = 0x2;
			  transmitc("Yellow led selected. Enter to continue or choose another led [g,y,r]: ");
		  } else if (cmd == 'r') {
			  edit_mode_led = 0x4;
			  transmitc("Red led selected. Enter to continue or choose another led [g,y,r]: ");
		  } else if (cmd == '\r') {
			  if (edit_mode_led == 0) {
				  transmitc("No led selected. Choose led for mode [g,y,r]: ");
			  } else {
				  state = ST_EDIT_WAIT_BRIGHT;
				  edit_mode_brightness = 40;
				  transmitc("Default led brightness 40%. Enter to continue or choose another brightness [1-9,0,+|-]: ");
			  }
		  } else {
			  if (edit_mode_led == 0) {
				  transmitc("Unknown command. Choose led for mode [g,y,r]: ");
			  } else {
				  char buf[128];
				  char *led = (edit_mode_led == 0x1 ? "Green" : edit_mode_led == 0x2 ? "Yellow" : "Red");
				  snprintf(buf, sizeof(buf), "Unknown command. %s led selected. Enter to continue or choose another led [g,y,r]: ", led);
				  transmitc(buf);
			  }
		  }
	  } else if (state == ST_EDIT_WAIT_BRIGHT) {
		  if (cmd >= '1' && cmd <= '9') {
			  edit_mode_brightness = (cmd - '0') * 10;
			  char buf[128];
			  snprintf(buf, sizeof(buf), "Led brightness %d%%. Enter to continue or choose another brightness [1-9,0,+|-]: ", edit_mode_brightness);
			  transmitc(buf);
		  } else if (cmd == '0') {
			  edit_mode_brightness = 100;
			  transmitc("Led brightness 100%. Enter to continue or choose another brightness [1-9,0,+|-]: ");
		  } else if (cmd == '+') {
			  edit_mode_brightness += 10;
			  if (edit_mode_brightness > 100) edit_mode_brightness = 100;
			  char buf[128];
			  snprintf(buf, sizeof(buf), "Led brightness %d%%. Enter to continue or choose another brightness [1-9,0,+|-]: ", edit_mode_brightness);
			  transmitc(buf);
		  } else if (cmd == '-') {
			  edit_mode_brightness -= 10;
			  if (edit_mode_brightness < 10) edit_mode_brightness = 10;
			  char buf[128];
			  snprintf(buf, sizeof(buf), "Led brightness %d%%. Enter to continue or choose another brightness [1-9,0,+|-]: ", edit_mode_brightness);
			  transmitc(buf);
		  } else if (cmd == '\r') {
			  state = ST_MAIN;
			  mode_leds[edit_mode] = edit_mode_led;
			  mode_brightness[edit_mode] = edit_mode_brightness;
			  transmitc("Exit edit mode");
			  print_mode_message();
		  } else {
			  char buf[128];
			  snprintf(buf, sizeof(buf), "Unknown command. Led brightness %d%%. Enter to continue or choose another brightness [1-9,0,+|-]: ", edit_mode_brightness);
			  transmitc(buf);
		  }
	  }
  }

  if (!hello_print) {
	  hello_print = true;
	  transmitc("\r\n\r\nProgramm started\r\n");
	  print_mode_message();
  }

  cur_mode = 0;

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  receive();

	  uint8_t read[2] = {0};
	  if (buf_pop(&receiveBuffer, read, sizeof(read))) {
		  char c = (char) read[0];
		  if (c == '\r') {
			  transmitc("\n");
		  } else {
			  transmitc("\r\n");
		  }
		  handle_command(c);
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }

  if (hello_print) {
  	  hello_print = false;
  	  transmitc("\r\nProgramm finished\r\n");
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 15;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
