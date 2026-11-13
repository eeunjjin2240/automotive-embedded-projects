/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "lwip.h"
#include "rng.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stm32f4xx_hal.h>
#include "examples_defines.h"
#include <example_selection.h>
#include <port.h>
#include <core_cm4.h>
#include "ESP8266_HAL.h"
#include "UartRingbuffer_multi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

SPI_HandleTypeDef   *hcurrent_active_spi=&hspi1;//Is the current active SPI pointer - 1 or 2
uint16_t            pin_io_active_spi=DW_NSS_Pin;//CS IO for SPI. Default SPI1
GPIO_PinState       SPI_CS_state=GPIO_PIN_RESET;//Determine the CS for the IO
host_using_spi_e    host_spi = SPI_1;

SPI_HandleTypeDef   hspi4;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

#define UNIT_TEST 0
#define WAIT_FOR_USB_CDC 0

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

extern example_ptr example_pointer;
extern volatile uint8_t tag_position_updated;
extern int unit_test_main(void);
extern uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn test_run_info()
 *
 * @brief  This gets run info from a test and sends it through virtual COM port.
 *
 * @param data - Message data, this data should be NULL string.
 *
 * output parameters
 *
 * no return value
 */
void test_run_info(unsigned char *data)
{
    uint16_t data_length;

    data_length = strlen((const char *)data);
//    CDC_Transmit_FS(data, data_length); /*Transmit the data through USB - Virtual port*/
//    CDC_Transmit_FS((uint8_t *)"\n\r", 2); /*Transmit end of line through USB - Virtual port*/
    HAL_UART_Transmit(&huart3, (uint8_t *)data, data_length, HAL_MAX_DELAY); /* Transmit the data via UART3 */
    HAL_UART_Transmit(&huart3, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);

}

void wifi_send(unsigned char *data)
{
    uint16_t data_length;

    data_length = strlen((const char *)data);
//    CDC_Transmit_FS(data, data_length); /*Transmit the data through USB - Virtual port*/
//    CDC_Transmit_FS((uint8_t *)"\n\r", 2); /*Transmit end of line through USB - Virtual port*/
    HAL_UART_Transmit(&huart2, (uint8_t *)data, data_length, HAL_MAX_DELAY); /* Transmit the data via UART3 */
    HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);

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

  build_examples();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_RNG_Init();
  MX_SPI1_Init();
  MX_LWIP_Init();
  MX_USB_DEVICE_Init();
  MX_USART2_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  DWT_Init();
  ESP_Init();
  /* disable IRQ from DW3000*/
  port_DisableEXT_IRQ();

  /*Sleep to wait for USB CDC to initialise with host*/
  #ifdef WAIT_FOR_USB_CDC
  Sleep(5000);
  #endif
  /*
   * DW_RESET_Pin has been configured by CubeMx as Exti0 line
   * Reconfigure the line as Input
   */
  setup_DWICRSTnIRQ(0);

  if (UNIT_TEST)
  {
	  unit_test_main();
  }
  else
  {
	  // Run the selected example as selected in example_selection.h
	  example_pointer();
  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 288;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM1_CC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
  /* ETH_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ETH_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ETH_IRQn);
}

/* USER CODE BEGIN 4 */
//responder에서 사용할 딜레이 초기화 함수
void DWT_Init(void) {
    // Enable DWT
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // Reset the cycle counter
    DWT->CYCCNT = 0;
    // Enable the cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
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
