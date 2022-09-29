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
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// 1-Wire struct containing GPIO infos
struct ONEWIRE_Config {
	GPIO_TypeDef* port; // Port to use
	uint16_t pin; // Pin number to use
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED2_INIT RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; \
        GPIOA->MODER |= GPIO_MODER_MODER5_0
#define LED2_ON (GPIOA->ODR |= 0x0020)
#define LED2_OFF (GPIOA->ODR &= ~0x0020)

// DS1820 1-Wire Commands
#define SKIP_ROM 			0xCC
#define CONVERT_T			0x44
#define READ_SCRATCHPAD		0xBE
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int __io_putchar(int ch) {
	while(((USART2->SR) &= (1<<7)) == 0);
	USART2->DR = ch;
}

// microseconds (µs) delay
void delayUs (uint32_t useconds)
{
	// Resetting timer's counter
	//TIM6->CNT = 0;
	__HAL_TIM_SET_COUNTER(&htim6, 0);

	// Works with an 1 MHz timer
	while(useconds > __HAL_TIM_GET_COUNTER(&htim6));
}

void gpio_set(struct ONEWIRE_Config *config) {
	config->port->ODR |= (config->pin);
}

void gpio_reset(struct ONEWIRE_Config *config) {
	config->port->ODR &= ~(config->pin);
}

uint8_t gpio_read(struct ONEWIRE_Config *config) {
	return (config->port->ODR) & (config->pin);
}

uint8_t ONEWIRE_Reset(struct ONEWIRE_Config *config)
{
	gpio_reset(config);
	delayUs(480);

	gpio_set(config);
	delayUs(70);

	// if Read == '0' => a slave answered
	uint8_t slave_answered = !!gpio_read(config);

	delayUs(410);

	/* Return values :
	 * 0 : no slave on the bus
	 * 1 : at least a slave on the bus
	 */
	return slave_answered;
}

void ONEWIRE_WriteBit0(struct ONEWIRE_Config *config)
{
	gpio_reset(config);
	delayUs(60);
	gpio_set(config);
	delayUs(10);
}

void ONEWIRE_WriteBit1(struct ONEWIRE_Config *config)
{
	gpio_reset(config);
	delayUs(6);

	gpio_set(config);
	delayUs(64);
}

uint8_t ONEWIRE_ReadBit(struct ONEWIRE_Config *config)
{
	gpio_reset(config);
	delayUs(6);

	gpio_set(config);
	delayUs(9);

	uint8_t read_bit = !!gpio_read(config);
	delayUs(55);

	return read_bit;
}

void ONEWIRE_WriteByte(struct ONEWIRE_Config *config, uint8_t byte_to_write)
{
	for(uint8_t i = 0; i < 8; i++) {
		switch((byte_to_write >> i) & 1) {
			case 0:
				ONEWIRE_WriteBit0(config);
				break;
			case 1:
				ONEWIRE_WriteBit1(config);
				break;
		}
	}
}

uint8_t ONEWIRE_ReadByte(struct ONEWIRE_Config *config)
{
	uint8_t read_byte = 0x00;
	for(uint8_t i = 0; i < 8; i++) {
		read_byte |= ONEWIRE_ReadBit(config) << i;
	}

	return read_byte;
}

uint8_t DS1820_GetTemp8Bits(struct ONEWIRE_Config *config)
{
	ONEWIRE_Reset(config);
	ONEWIRE_WriteByte(config, SKIP_ROM);
	ONEWIRE_WriteByte(config, CONVERT_T);

	delayUs(10000); //delaying 10 ms = 10 000 µs

	ONEWIRE_Reset(config);
	ONEWIRE_WriteByte(config, SKIP_ROM);
	ONEWIRE_WriteByte(config, READ_SCRATCHPAD);

	// Reading DS1820's temp (Scratchpad's byte 0)
	uint8_t ds12820_temp_8_bits = ONEWIRE_ReadByte(config);

	return ds12820_temp_8_bits;
}

int16_t DS1820_GetTemp16Bits(struct ONEWIRE_Config *config)
{
	ONEWIRE_Reset(config);
	ONEWIRE_WriteByte(config, SKIP_ROM);
	ONEWIRE_WriteByte(config, CONVERT_T);

	delayUs(10000); //delaying 10 ms = 10 000 µs

	ONEWIRE_Reset(config);
	ONEWIRE_WriteByte(config, SKIP_ROM);
	ONEWIRE_WriteByte(config, READ_SCRATCHPAD);

	// Reading DS1820's temp first part (Scratchpad's byte 0)
	int16_t ds12820_temp_full_res = ONEWIRE_ReadByte(config);

	// Reading DS1820's temp last part (Scratchpad's byte 1)
	ds12820_temp_full_res += ONEWIRE_ReadByte(config) << 8;

	return ds12820_temp_full_res; // Don't forget to convert it to the selected res
}

float convert_temperature_using_res(int16_t temp_to_convert, float res)
{
	return temp_to_convert * res;
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
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

	/* TIMER PARAMETERS :
		We're using TIM1
		Internal Clock = 84 MHz
		We want 1 µs => PRESCALER = 84
		Resulting timer frequency => CLK_SPEED / PRESCALER = 1 MHz = 1 µs
	*/
  __HAL_TIM_ENABLE(&htim6);
	// Configure 1-Wire bus
	struct ONEWIRE_Config config_ow = {GPIOA, GPIO_PIN_8};

	// 1-Wire GPIO Config
	//RCC Register // Port clocking
	//GPIOA->MODER |= GPIO_MODER_MODER6_0; 	// Output
	/*
	GPIOA->OTYPER |= (1 << GPIO_OTYPER_OT9_Pos); 	// Open Drain
	GPIOA->PUPDR |= (1 << GPIO_PUPDR_PUPD9_Pos); 	// Pull-up Enabled
*/
  // PA5 LED Config
  LED2_INIT;
  LED2_OFF;

	// Enable TIM6
	//TIM6->CR1 |=  TIM_CR1_CEN;



	// Valid values for next define : FIRST_PHASE, SECOND_PHASE, THIRD_PHASE
	#define SECOND_PHASE

	struct ONEWIRE_Config led = {GPIOA, GPIO_PIN_5};

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

#ifdef FIRST_PHASE
		// 1st step : Hello World => validate first 1-Wire bus implementation

		// Sets PA5 LED ON for 1 second if a device is on this 1-Wire Bus
		if(!ONEWIRE_Reset(&config_ow)) {
			LED2_ON;
		} else {
			LED2_OFF;
		}

		for(uint8_t i = 0; i<20; i++) { // 1 second delay
			delayUs(50000);
		}

#elif defined(SECOND_PHASE)
		// 2nd step : Getting 8 bit temperature from DS1820 => validate communication w/ DS1820
		// Print received temp on UART

		printf("Acquisition\n");
		// Get DS1820's temperature (restricted to 8 bits)
		uint8_t temp_8_bits = DS1820_GetTemp8Bits(&config_ow); // unit : depends on resolution !

		printf("\n%d\n", temp_8_bits);

		for(uint8_t i = 0; i<20; i++) { // 1 second delay
			delayUs(50000);
		}


#elif defined(THIRD_PHASE)
		// 3rd step : Getting temp full res after button press ?
		// Mean of the temp every 10 seconds ?
		// Alarm Signaling ? (ref in datasheet)

		// Get DS1820's temperature (full resolution)
		int16_t temp_16_bits = DS1820_GetTemp16Bits(config); // unit : depends on resolution !

		// You need to convert temp_16_bits before printing it !
		float current_temperature = convert_temperature_using_res(temp_16_bits, 0.5);

		// Printing to UART

		HAL_Delay(2000);
#else
#error Please define a valid macro name for phase !
#endif
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 83;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
