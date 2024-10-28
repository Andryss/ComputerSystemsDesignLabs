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
#include "usart.h"
#include "gpio.h"
#include "buf.h"
#include "btn.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MODES 8
#define MODE_STEPS 8
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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
_Bool is_interrupts = false;

void enable_interrupts() {
	HAL_NVIC_EnableIRQ(USART6_IRQn);
	is_interrupts = true;
}

void disable_interrupts() {
	HAL_UART_AbortReceive(&huart6);
	HAL_Delay(10);
	HAL_NVIC_DisableIRQ(USART6_IRQn);
	is_interrupts = false;
}

struct Buffer receiveBuffer = {0};
struct Buffer transmitBuffer = {0};

uint8_t char_buf[1] = {'\0'};

void transmit(uint8_t* data, size_t size) {
	if (is_interrupts) {
		if (HAL_UART_Transmit_IT(&huart6, data, size) == HAL_BUSY) {
			buf_push(&transmitBuffer, data, size);
		}
	} else {
		HAL_UART_Transmit(&huart6, data, size, 100);
	}
}

void transmitc(char* data) {
	transmit((uint8_t*) data, strlen(data));
}

void receive() {
	if (is_interrupts) {
		HAL_UART_Receive_IT(&huart6, char_buf, sizeof(char_buf));
	} else {
		if (HAL_UART_Receive(&huart6, char_buf, sizeof(char_buf), 0) == HAL_OK) {
			buf_push(&receiveBuffer, char_buf, sizeof(char_buf));
			transmit(char_buf, sizeof(char_buf));
		}
	}
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
  /* USER CODE BEGIN 2 */
  uint32_t garland_modes[MODES][MODE_STEPS] = {
		  { 0x1, 0x2, 0x1, 0x2, -1, -1, -1, -1 },
		  { 0x3, 0x0, 0x3, 0x0, -1, -1, -1, -1 },
		  { 0x1, 0x2, 0x4, 0x0, -1, -1, -1, -1 },
		  { 0x2, 0x4, 0x2, 0x4, -1, -1, -1, -1 },
		  { -1, -1, -1, -1, -1, -1, -1, -1 },
		  { -1, -1, -1, -1, -1, -1, -1, -1 },
		  { -1, -1, -1, -1, -1, -1, -1, -1 },
		  { -1, -1, -1, -1, -1, -1, -1, -1 },
  };
  uint32_t garland_freq[MODES] = { 400, 400, 1000, 100, 500, 500, 500, 500 };
  size_t modes_steps[MODES] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  size_t cur_mode = 0;

  int should_change_mode = 0;
  uint16_t btn_state;

  uint16_t state = 0; // waiting for command
  uint32_t new_mode_steps[MODE_STEPS] = { -1, -1, -1, -1, -1, -1, -1, -1 };
  size_t new_mode_num = 4;

  char stdin[256] = {0};
  size_t stdinp = 0;

  bool is_digit(char c) {
	  return c >= '0' && c <= '9';
  }

  void validate_command() {
	  if (state == 0) {
		  if (stdinp >= 4 && (strncmp(stdin, "new ", 4) == 0)) {
			  size_t i = 0;
			  char color = *(stdin + 4 + i);
			  bool unknown_led = false;
			  while (i < MODE_STEPS && color != '\r') {
				  if (color == 'g') new_mode_steps[i] = 0x1;
				  else if (color == 'y') new_mode_steps[i] = 0x2;
				  else if (color == 'r') new_mode_steps[i] = 0x4;
				  else if (color == 'n') new_mode_steps[i] = 0x0;
				  else {
					  unknown_led = true;
					  transmitc("Unknown led\r\n");
					  break;
				  }
				  i++;
				  color = *(stdin + 4 + i);
			  }
			  if (!unknown_led) {
				  while (i < MODE_STEPS) new_mode_steps[i++] = -1;
				  state = 1; // wait for new mode freq input
				  transmitc("Input new mode freq (0-9):\r\n");
			  }
		  }
		  else if (stdinp >= 4 && (strncmp(stdin, "set ", 4) == 0) && is_digit(stdin[4])) {
			  size_t mode = stdin[4] - '0';
			  if (mode >= MODES) {
				  transmitc("Unknown mode\r\n");
			  } else {
				  cur_mode = mode;
				  transmitc("OK\r\n");
			  }
		  }
		  else if (stdinp >= 17 && (strncmp(stdin, "set interrupts on", 17) == 0)) {
			  enable_interrupts();
			  transmitc("OK\r\n");
		  }
		  else if (stdinp >= 18 && (strncmp(stdin, "set interrupts off", 18) == 0)) {
			  disable_interrupts();
			  transmitc("OK\r\n");
		  }
		  else {
			  transmitc("Unknown command\r\n");
		  }
	  } else if (state == 1) {
		  if (stdinp >= 1 && is_digit(stdin[0])) {
			  garland_freq[new_mode_num] = (stdin[0] - '0' + 1) * 100;
			  for (size_t i = 0; i < MODE_STEPS; i++) {
				  garland_modes[new_mode_num][i] = new_mode_steps[i];
			  }
			  state = 0; // wait for command
			  new_mode_num = (new_mode_num + 1) % MODES;
			  if (new_mode_num == 0) new_mode_num = 4;
			  transmitc("OK\r\n");
		  }
		  else {
			  transmitc("Unknown freq\r\n");
		  }
	  }
	  transmitc("> ");
	  stdinp = 0;
  }

  void prevalidate_command() {
	  if (state == 0) {
		  if (stdinp >= 12 && (strncmp(stdin, "new ", 4) == 0)) {
			  transmitc("\r\n");
			  validate_command();
		  }
	  } else if (state == 1) {
		  if (stdinp >= 1) {
			  transmitc("\r\n");
			  validate_command();
		  }
	  }
  }

  transmitc("Ready to work\r\n> ");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint32_t cur_state = garland_modes[cur_mode][modes_steps[cur_mode]];

	  if (cur_state == -1) {
		  while (cur_state == -1) {
			  modes_steps[cur_mode] = 0;
			  cur_state = garland_modes[cur_mode][0];
			  if (cur_state == -1) cur_mode = (cur_mode + 1) % MODES;
		  }
	  }

	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

	  if ((cur_state & 0x1) != 0) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
	  if ((cur_state & 0x2) != 0) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
	  if ((cur_state & 0x4) != 0) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);

	  uint32_t wait_ticks = garland_freq[cur_mode];
	  uint32_t ticks = HAL_GetTick();
	  while ((HAL_GetTick() - ticks) < wait_ticks) {

		  receive();

		  uint8_t read[2] = {0};
		  if (buf_pop(&receiveBuffer, read, sizeof(read))) {
			  char c = (char) read[0];
			  if (stdinp == sizeof(stdin)) {
				  transmitc("\r\nInput buffer overflow\r\n> ");
				  stdinp = 0;
			  } else {
				  stdin[stdinp++] = c;
				  if (c == '\r') {
					  transmitc("\n");
					  validate_command();
				  } else {
					  prevalidate_command();
				  }
			  }
		  }

		  BTN_GetState(&btn_state);
		  if (BTN_IsSet(&btn_state, BTN_DOWN_EVENT)) {
			  should_change_mode = 1;
			  break;
		  }
	  }

	  if (should_change_mode == 1) {
		  should_change_mode = 0;
		  cur_mode = (cur_mode + 1) % MODES;
		  continue;
	  }

	  modes_steps[cur_mode] = (modes_steps[cur_mode] + 1) % MODE_STEPS;
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
