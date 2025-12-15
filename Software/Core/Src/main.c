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
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "usbd_cdc_if.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void AD9106_WriteReg(uint16_t reg, uint16_t data)
{
  uint8_t buf[4] = {0};
  buf[0] = (reg >> 8) & 0x7F; // bit15=0 (write)
  buf[1] = reg & 0xFF;
  buf[2] = (data >> 8);
  buf[3] = (data & 0xFF);
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, buf, 4, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

  HAL_Delay(50);
}

uint16_t AD9106_ReadReg(uint16_t reg)
{
  uint8_t tx[4];
  uint8_t rx[4];

  // bit15 = 1 → read
  tx[0] = ((reg >> 8) & 0x7F) | 0x80;
  tx[1] = reg & 0xFF;
  tx[2] = 0x00; // dummy bytes
  tx[3] = 0x00;

  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, tx, rx, 4, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

  // Combine received data bytes (the last two bytes are the register data)
  uint16_t data = ((uint16_t)rx[2] << 8) | rx[3];
  return data;
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(FUNC_RESET_GPIO_Port, FUNC_RESET_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  HAL_Delay(500);

  HAL_GPIO_WritePin(FUNC_RESET_GPIO_Port, FUNC_RESET_Pin, GPIO_PIN_RESET);

  HAL_Delay(500);

  HAL_GPIO_WritePin(FUNC_RESET_GPIO_Port, FUNC_RESET_Pin, GPIO_PIN_SET);

  uint8_t data[1] = {1};
  HAL_SPI_Transmit(&hspi1, data, 1, 100);

  HAL_Delay(100);

  // 2. Configure registers
  /*
    // (a) Power configuration: disable DAC1, DAC2, DAC4 (sleep), keep DAC3 active
    AD9106_WriteReg(0x01, 0b0000000000001101); // DAC1_SLEEP, DAC2_SLEEP, DAC4_SLEEP = 1

    HAL_Delay(10);

    AD9106_ReadReg(0x01);

    HAL_Delay(10);
    // (b) DDS tuning word (sets output frequency)
    AD9106_WriteReg(0x3E, 0x0000); // DDS_TW32 (MSB)

    HAL_Delay(10);
    AD9106_WriteReg(0x3F, 0xFF00); // DDS_TW1  (LSB) → Tuning word = 0x00A300

    HAL_Delay(10);

    AD9106_ReadReg(0x3F);
    HAL_Delay(10);
    // (c) Select waveform source for DAC3 = DDS
    AD9106_WriteReg(0x26, 0x0300); // PRESTORE_SEL3 = 3 (DDS), WAVE_SEL3 = 0 (prestored waveform)
    HAL_Delay(10);
    // (d) Set unity digital gain for DAC3
    AD9106_WriteReg(0x33, 0x0400); // DAC3_DGAIN = +1.0
    HAL_Delay(10);
    // (f) Enable pattern generator
    AD9106_WriteReg(0x1E, 0x0001); // RUN = 1
    HAL_Delay(10);
    AD9106_ReadReg(0x1E);
    HAL_Delay(10);
    // (e) Update all configuration shadow registers
    AD9106_WriteReg(0x1D, 0x0001); // RAMUPDATE = 1
    HAL_Delay(10);

    */
  uint16_t AD910x_SPI_Register_Addresses[66] = {0x0000, 0x0001, 0x0002, 0x0003, 0x0004, 0x0005, 0x0006, 0x0007, 0x0008, 0x0009, 0x000a, 0x000b, 0x000c, 0x000d, 0x000e, 0x001f, 0x0020, 0x0022, 0x0023, 0x0024, 0x0025, 0x0026, 0x0027, 0x0028, 0x0029, 0x002a, 0x002b, 0x002c, 0x002d, 0x002e, 0x002f, 0x0030, 0x0031, 0x0032, 0x0033, 0x0034, 0x0035, 0x0036, 0x0037, 0x003e, 0x003f, 0x0040, 0x0041, 0x0042, 0x0043, 0x0044, 0x0045, 0x0047, 0x0050, 0x0051, 0x0052, 0x0053, 0x0054, 0x0055, 0x0056, 0x0057, 0x0058, 0x0059, 0x005a, 0x005b, 0x005c, 0x005d, 0x005e, 0x005f, 0x001e, 0x001d};
  uint16_t AD9106_example3_regval[66] = {0x0000, 0x0e00, 0x0000, 0x0000, 0x4000, 0x4000, 0x4000, 0x4000, 0x0000, 0x1f00, 0x1f00, 0x1f00, 0x1f00, 0x0000, 0x0000, 0x0000, 0x000e, 0x0000, 0x0000, 0x0000, 0x0000, 0x3232, 0x3232, 0x0111, 0xffff, 0x0101, 0x0101, 0x0003, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x4000, 0x2000, 0x2000, 0x4000, 0x0001, 0x0200, 0x0a3d, 0x7100, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x07d0, 0x0000, 0x0000, 0x0100, 0x03e8, 0x0000, 0x0000, 0x0100, 0x0bb8, 0x0000, 0x0000, 0x0100, 0x0fa0, 0x0000, 0x0000, 0x0100, 0x0001, 0x0001};
  uint16_t AD9106_example4_regval[66] = {0x0000, 0x0e00, 0x0000, 0x0000, 0x4000, 0x4000, 0x4000, 0x4000, 0x0000, 0x1f00, 0x1f00, 0x1f00, 0x1f00, 0x0000, 0x0000, 0x0000, 0x000e, 0x0000, 0x0000, 0x0000, 0x0000, 0x1212, 0x1232, 0x0121, 0xffff, 0x0101, 0x0101, 0x0003, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x4000, 0x4000, 0x4000, 0x4000, 0x1011, 0x0600, 0x1999, 0x9a00, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x07d0, 0x0000, 0x0000, 0x0001, 0x03e8, 0x0000, 0x0000, 0x0001, 0x03e8, 0x0000, 0x0000, 0x0001, 0x0fa0, 0x0000, 0x0000, 0x16ff, 0x0001, 0x0001};
  uint16_t AD9106_example6_regval[66] = {0x0000, 0x0e00, 0x0000, 0x0000, 0x4000, 0x4000, 0x4000, 0x4000, 0x0000, 0x1f00, 0x1f00, 0x1f00, 0x1f00, 0x0000, 0x0000, 0x0000, 0x000e, 0x0000, 0x0000, 0x0000, 0x0000, 0x1212, 0x1232, 0x0111, 0xffff, 0x0101, 0x0101, 0x0003, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x4000, 0x4000, 0x4000, 0x4000, 0x0001, 0x7e00, 0x0750, 0x7500, 0x0000, 0x0000, 0x0000, 0x0000, 0x0002, 0x0000, 0x0000, 0x2710, 0x0000, 0x0000, 0x0001, 0x0000, 0x0000, 0x0000, 0x0001, 0x1770, 0x0000, 0x0000, 0x0001, 0x0fa0, 0x0000, 0x0000, 0x7fff, 0x0001, 0x0001};

  for (int i = 0; i < 66; i++)
  {
    AD9106_WriteReg(AD910x_SPI_Register_Addresses[i], AD9106_example4_regval[i]);
    AD9106_ReadReg(AD910x_SPI_Register_Addresses[i]);

    HAL_Delay(10);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint8_t Buf[20] = {"Hello World"};
  while (1)
  {

    CDC_Transmit_FS(Buf, 20);
    HAL_Delay(100);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
   */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
   */
  HAL_RCCEx_EnableMSIPLLMode();
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
#ifdef USE_FULL_ASSERT
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
