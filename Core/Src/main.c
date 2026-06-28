/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "usb_device.h"
#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define IMU_I2C_TIMEOUT_MS      20U
#define IMU_WHOAMI_REG_MPU      0x75U
#define IMU_WHOAMI_REG_LSM6     0x0FU
#define IMU_BRINGUP_ENABLE      0U
#define IMU_SPI_TIMEOUT_MS      20U
#define IMU_CS_PORT             GPIOE
#define IMU_CS_PIN              GPIO_PIN_4
#define IMU_SCK_PORT            GPIOE
#define IMU_SCK_PIN             GPIO_PIN_2
#define IMU_MISO_PORT           GPIOE
#define IMU_MISO_PIN            GPIO_PIN_5
#define IMU_MOSI_PORT           GPIOE
#define IMU_MOSI_PIN            GPIO_PIN_6
#define IMU_WHOAMI_REG_BMI270   0x00U
#define IMU_REG_ACCEL_DATA_X1   0x1FU
#define IMU_REG_PWR_MGMT0       0x4EU
#define IMU_REG_GYRO_CONFIG0    0x4FU
#define IMU_REG_ACCEL_CONFIG0   0x50U

#define IMU_UART_INSTANCE       UART5
#define IMU_UART_BAUDRATE       115200U
#define IMU_UART_TX_PORT        GPIOB
#define IMU_UART_TX_PIN         GPIO_PIN_13
#define IMU_UART_TX_AF          GPIO_AF14_UART5

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

volatile uint8_t g_imu_found = 0U;
volatile uint8_t g_imu_bus_id = 0U;
volatile uint8_t g_imu_whoami = 0U;
volatile uint32_t g_imu_probe_error = 0U;
volatile uint32_t g_imu_status = 0U;
volatile int32_t g_roll_cd = 0;
volatile int32_t g_pitch_cd = 0;
volatile int32_t g_yaw_cd = 0;
volatile int16_t g_gyro_x_raw = 0;
volatile int16_t g_gyro_y_raw = 0;
volatile int16_t g_gyro_z_raw = 0;
volatile int16_t g_accel_x_raw = 0;
volatile int16_t g_accel_y_raw = 0;
volatile int16_t g_accel_z_raw = 0;

/* Gyro bias calibration */
static int32_t g_gyro_bias_x = 0;
static int32_t g_gyro_bias_y = 0;
static int32_t g_gyro_bias_z = 0;
static uint8_t g_gyro_calibrated = 0U;

static uint8_t g_imu_spi_mode = 3U; /* 3=mode3, 0=mode0 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static uint8_t MX_SPI4_Init(void);
static void USB_WriteString(const char *text);
static uint8_t IMU_Probe(void);
static uint8_t IMU_ReadWhoAmI(uint8_t reg, uint8_t *whoami);
static uint8_t IMU_ReadReg(uint8_t reg, uint8_t *value);
static uint8_t IMU_WriteReg(uint8_t reg, uint8_t value);
static uint8_t IMU_ReadBurst(uint8_t startReg, uint8_t *data, uint8_t len);
static uint8_t IMU_Config(void);
static uint8_t IMU_ReadRaw(int16_t *gx, int16_t *gy, int16_t *gz);
static uint8_t IMU_ReadAccel(int16_t *ax, int16_t *ay, int16_t *az);
static uint8_t IMU_SpiTransfer(uint8_t tx);
static uint8_t IMU_SpiTransferMode0(uint8_t tx);
static uint8_t IMU_SpiTransferMode3(uint8_t tx);
static void IMU_SpiDelay(void);
static void MX_UART4_Init(void);
static void UART4_WriteString(const char *text);
static void Debug_Attach_Window(void);
static uint32_t Blink_Delay_Count(void);
/* USER CODE BEGIN PFP */

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
  /* MCU Configuration */
  HAL_Init();
  SystemClock_Config();
  MPU_Config();
  
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  
  uint32_t counter = 0;
  uint8_t usb_buffer[64];
  
  /* Main loop */
  while (1)
  {
    /* LED toggle */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_Delay(500);
    
    /* Send counter to USB */
    int len = snprintf((char*)usb_buffer, sizeof(usb_buffer), "Counter: %lu\r\n", counter++);
    if (len > 0 && len < (int)sizeof(usb_buffer)) {
      CDC_Transmit_FS(usb_buffer, (uint16_t)len);
    }
    
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_Delay(500);
  }
  
  return 0;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Skip power config - already done in SystemInit() */
  /* HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY); */
  /* __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3); */
  /* timeout loop for VOSRDY skipped */

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  
  /* PLL setup for USB: HSI 64MHz -> PLL -> 48MHz USB (Q output), 120MHz system (P output) */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;    /* 64 MHz / 8 = 8 MHz reference */
  RCC_OscInitStruct.PLL.PLLN = 60;   /* 8 MHz * 60 = 480 MHz VCO */
  RCC_OscInitStruct.PLL.PLLP = 4;    /* 480 MHz / 4 = 120 MHz (system) */
  RCC_OscInitStruct.PLL.PLLQ = 10;   /* 480 MHz / 10 = 48 MHz (USB) */
  RCC_OscInitStruct.PLL.PLLR = 2;    /* 480 MHz / 2 = 240 MHz (unused) */
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /* PC2_C is behind an internal analog switch on H7 dual-pad pins. */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC2, SYSCFG_SWITCH_PC2_CLOSE);

  /* Configure PC2 as push-pull output. */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);

  /* IMU chip-select (SPI4) */
  HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = IMU_CS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(IMU_CS_PORT, &GPIO_InitStruct);

  /* IMU SPI bit-bang pins (SPI4 mapping on KAKUTEH7): PE2/PE5/PE6 */
  GPIO_InitStruct.Pin = IMU_SCK_PIN | IMU_MOSI_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = IMU_MISO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  HAL_GPIO_WritePin(IMU_SCK_PORT, IMU_SCK_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IMU_MOSI_PORT, IMU_MOSI_PIN, GPIO_PIN_RESET);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief Send string to USB CDC device (virtual COM port)
  * @param text: null-terminated string to transmit
  * @retval None
  */
static void USB_WriteString(const char *text)
{
  if (text == NULL)
  {
    return;
  }

  uint16_t len = 0;
  while ((text[len] != '\0') && (len < 256U))
  {
    len++;
  }

  if (len > 0)
  {
    /* CDC_Transmit_FS((uint8_t *)text, len); */ /* Disabled: USB not yet initialized */
  }
}

static uint8_t MX_SPI4_Init(void)
{
  /* Bit-banged SPI uses GPIO only; pins are configured in MX_GPIO_Init. */
  g_imu_probe_error = 0U;

  return 1U;
}

static uint8_t IMU_ReadWhoAmI(uint8_t reg, uint8_t *whoami)
{
  uint8_t valueMode3;
  uint8_t valueMode0;

  /* Try mode 3 first (common for MPU/ICM on FCs), then mode 0 fallback. */
  HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IMU_SCK_PORT, IMU_SCK_PIN, GPIO_PIN_SET);
  (void)IMU_SpiTransferMode3((uint8_t)(reg | 0x80U));
  valueMode3 = IMU_SpiTransferMode3(0x00U);
  HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);

  HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IMU_SCK_PORT, IMU_SCK_PIN, GPIO_PIN_RESET);
  (void)IMU_SpiTransferMode0((uint8_t)(reg | 0x80U));
  valueMode0 = IMU_SpiTransferMode0(0x00U);
  HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);

  if ((valueMode3 != 0x00U) && (valueMode3 != 0xFFU))
  {
    g_imu_spi_mode = 3U;
    *whoami = valueMode3;
    return 1U;
  }

  if ((valueMode0 != 0x00U) && (valueMode0 != 0xFFU))
  {
    g_imu_spi_mode = 0U;
    *whoami = valueMode0;
    return 1U;
  }

  g_imu_probe_error = 0xE303U; /* no valid SPI response */
  return 0U;
}

static uint8_t IMU_Probe(void)
{
  uint8_t whoami = 0U;

  if (IMU_ReadWhoAmI(IMU_WHOAMI_REG_MPU, &whoami) != 0U)
  {
    if ((whoami == 0x68U) || (whoami == 0x70U) || (whoami == 0x71U) ||
        (whoami == 0x42U) || (whoami == 0x47U))
    {
      g_imu_bus_id = 4U; /* bus marker: SPI4 */
      g_imu_whoami = whoami;
      return 1U;
    }
  }

  if (IMU_ReadWhoAmI(IMU_WHOAMI_REG_BMI270, &whoami) != 0U)
  {
    if (whoami == 0x24U)
    {
      g_imu_bus_id = 4U; /* bus marker: SPI4 */
      g_imu_whoami = whoami;
      return 1U;
    }
  }

  if (g_imu_probe_error == 0U)
  {
    g_imu_probe_error = 0xE302U; /* SPI responded but WHO_AM_I did not match supported IDs */
  }

  return 0U;
}

static uint8_t IMU_SpiTransferMode0(uint8_t tx)
{
  uint8_t rx = 0U;

  for (uint32_t bit = 0U; bit < 8U; bit++)
  {
    HAL_GPIO_WritePin(IMU_MOSI_PORT,
                      IMU_MOSI_PIN,
                      ((tx & 0x80U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    tx <<= 1;

    IMU_SpiDelay();
    HAL_GPIO_WritePin(IMU_SCK_PORT, IMU_SCK_PIN, GPIO_PIN_SET);
    IMU_SpiDelay();

    rx <<= 1;
    if (HAL_GPIO_ReadPin(IMU_MISO_PORT, IMU_MISO_PIN) == GPIO_PIN_SET)
    {
      rx |= 1U;
    }

    HAL_GPIO_WritePin(IMU_SCK_PORT, IMU_SCK_PIN, GPIO_PIN_RESET);
  }

  return rx;
}

static uint8_t IMU_SpiTransfer(uint8_t tx)
{
  if (g_imu_spi_mode == 0U)
  {
    return IMU_SpiTransferMode0(tx);
  }

  return IMU_SpiTransferMode3(tx);
}

static uint8_t IMU_ReadReg(uint8_t reg, uint8_t *value)
{
  HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET);
  (void)IMU_SpiTransfer((uint8_t)(reg | 0x80U));
  *value = IMU_SpiTransfer(0x00U);
  HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);

  if ((*value == 0x00U) || (*value == 0xFFU))
  {
    return 0U;
  }

  return 1U;
}

static uint8_t IMU_WriteReg(uint8_t reg, uint8_t value)
{
  HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET);
  (void)IMU_SpiTransfer((uint8_t)(reg & 0x7FU));
  (void)IMU_SpiTransfer(value);
  HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);
  return 1U;
}

static uint8_t IMU_ReadBurst(uint8_t startReg, uint8_t *data, uint8_t len)
{
  if ((data == NULL) || (len == 0U))
  {
    return 0U;
  }

  HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET);
  (void)IMU_SpiTransfer((uint8_t)(startReg | 0x80U));

  for (uint8_t i = 0U; i < len; i++)
  {
    data[i] = IMU_SpiTransfer(0x00U);
  }

  HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);
  return 1U;
}

static uint8_t IMU_Config(void)
{
  uint8_t verify = 0U;

  (void)IMU_WriteReg(IMU_REG_PWR_MGMT0, 0x0FU);    /* accel+gyro LN mode */
  HAL_Delay(2U);
  (void)IMU_WriteReg(IMU_REG_GYRO_CONFIG0, 0x06U); /* 2000dps, 1kHz */
  (void)IMU_WriteReg(IMU_REG_ACCEL_CONFIG0, 0x06U);/* 16g, 1kHz */
  HAL_Delay(2U);

  if (IMU_ReadReg(IMU_REG_PWR_MGMT0, &verify) == 0U)
  {
    g_imu_probe_error = 0xE305U;
    return 0U;
  }

  if ((verify & 0x0FU) != 0x0FU)
  {
    g_imu_probe_error = 0xE306U;
    return 0U;
  }

  return 1U;
}

static uint8_t IMU_ReadRaw(int16_t *gx, int16_t *gy, int16_t *gz)
{
  uint8_t data[6];

  if ((gx == NULL) || (gy == NULL) || (gz == NULL))
  {
    return 0U;
  }

  /* Read gyro data only (starting at register 0x25, 6 bytes: GX, GY, GZ) */
  if (IMU_ReadBurst(0x25U, data, (uint8_t)sizeof(data)) == 0U)
  {
    g_imu_probe_error = 0xE307U;
    return 0U;
  }

  *gx = (int16_t)((((uint16_t)data[0]) << 8) | (uint16_t)data[1]);
  *gy = (int16_t)((((uint16_t)data[2]) << 8) | (uint16_t)data[3]);
  *gz = (int16_t)((((uint16_t)data[4]) << 8) | (uint16_t)data[5]);
  return 1U;
}

static uint8_t IMU_ReadAccel(int16_t *ax, int16_t *ay, int16_t *az)
{
  uint8_t data[6];

  if ((ax == NULL) || (ay == NULL) || (az == NULL))
  {
    return 0U;
  }

  /* Read accel data (register 0x1F, 6 bytes: AX, AY, AZ) */
  if (IMU_ReadBurst(0x1FU, data, (uint8_t)sizeof(data)) == 0U)
  {
    return 0U;
  }

  *ax = (int16_t)((((uint16_t)data[0]) << 8) | (uint16_t)data[1]);
  *ay = (int16_t)((((uint16_t)data[2]) << 8) | (uint16_t)data[3]);
  *az = (int16_t)((((uint16_t)data[4]) << 8) | (uint16_t)data[5]);
  return 1U;
}

static uint8_t IMU_SpiTransferMode3(uint8_t tx)
{
  uint8_t rx = 0U;

  for (uint32_t bit = 0U; bit < 8U; bit++)
  {
    HAL_GPIO_WritePin(IMU_MOSI_PORT,
                      IMU_MOSI_PIN,
                      ((tx & 0x80U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    tx <<= 1;

    HAL_GPIO_WritePin(IMU_SCK_PORT, IMU_SCK_PIN, GPIO_PIN_RESET);
    IMU_SpiDelay();

    rx <<= 1;
    if (HAL_GPIO_ReadPin(IMU_MISO_PORT, IMU_MISO_PIN) == GPIO_PIN_SET)
    {
      rx |= 1U;
    }

    HAL_GPIO_WritePin(IMU_SCK_PORT, IMU_SCK_PIN, GPIO_PIN_SET);
    IMU_SpiDelay();
  }

  return rx;
}

static void IMU_SpiDelay(void)
{
  for (volatile uint32_t i = 0U; i < 10U; i++)
  {
    __NOP();
  }
}

static void MX_UART4_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  uint32_t pclk1Hz;

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_UART5_CLK_ENABLE();

  GPIO_InitStruct.Pin = IMU_UART_TX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = IMU_UART_TX_AF;
  HAL_GPIO_Init(IMU_UART_TX_PORT, &GPIO_InitStruct);

  IMU_UART_INSTANCE->CR1 = 0U;
  IMU_UART_INSTANCE->CR2 = 0U;
  IMU_UART_INSTANCE->CR3 = 0U;

  pclk1Hz = HAL_RCC_GetPCLK2Freq();  /* UART5 is on APB2 */
  IMU_UART_INSTANCE->BRR = (pclk1Hz + (IMU_UART_BAUDRATE / 2U)) / IMU_UART_BAUDRATE;
  IMU_UART_INSTANCE->CR1 = USART_CR1_TE;
  IMU_UART_INSTANCE->CR1 |= USART_CR1_UE;
}

static void UART4_WriteString(const char *text)
{
  if (text == NULL)
  {
    return;
  }

  while (*text != '\0')
  {
    while ((IMU_UART_INSTANCE->ISR & USART_ISR_TXE_TXFNF) == 0U)
    {
    }

    IMU_UART_INSTANCE->TDR = (uint8_t)(*text);
    text++;
  }
}

static void Debug_Attach_Window(void)
{
  /* Give the probe several seconds and exit early once debugger is attached. */
  for (volatile uint32_t i = 0; i < 120000000UL; i++)
  {
    if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0U)
    {
      break;
    }
    __NOP();
  }
}

static uint32_t Blink_Delay_Count(void)
{
  if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0U)
  {
    return 16000000UL;
  }

  return 8000000UL;
}

/* USER CODE END 4 */

 /* MPU Configuration */

__attribute__((unused)) static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
