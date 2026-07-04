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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdlib.h>
#include <math.h>
#include "usbd_cdc_if.h"
#include "Fusion.h"

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
#define IMU_ACCEL_LSB_PER_G     2048.0f
#define IMU_GYRO_DPS_PER_LSB    (2000.0f / 32768.0f)

#define IMU_ALIGN_CW0           0U
#define IMU_ALIGN_CW90          1U
#define IMU_ALIGN_CW180         2U
#define IMU_ALIGN_CW270         3U
/* Board-to-sensor alignment mapping (matches verified physical orientation). */
#define IMU_SENSOR_ALIGNMENT     IMU_ALIGN_CW270

#define CRSF_FRAME_TYPE_RC_CHANNELS     0x16U
#define CRSF_SYNC_BYTE                  0xC8U
#define CRSF_ADDRESS_BROADCAST          0x00U
#define CRSF_ADDRESS_RADIO_TX           0xEAU
#define CRSF_ADDRESS_CRSF_RX            0xECU
#define CRSF_ADDRESS_CRSF_TX            0xEEU
#define CRSF_MAX_FRAME_BYTES            64U
#define CRSF_RC_CHANNEL_COUNT           16U
#define CRSF_RC_PAYLOAD_BYTES           22U
#define CRSF_FRAME_GAP_RESET_MS         3U
#define CRSF_RX_FIFO_SIZE               256U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;

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

/* Fused Euler angles from complementary filter */
static float g_roll_fused = 0.0f;
static float g_pitch_fused = 0.0f;
static float g_yaw_fused = 0.0f;
static float g_q0 = 1.0f;
static float g_q1 = 0.0f;
static float g_q2 = 0.0f;
static float g_q3 = 0.0f;
static FusionAhrs g_fusion_ahrs;
static uint8_t g_fusion_ready = 0U;

static uint8_t g_imu_spi_mode = 3U; /* 3=mode3, 0=mode0 */
static uint8_t g_usb_ready = 0U;
static uint32_t g_telemetry_sequence = 0U;

static volatile uint16_t g_rc_channels_raw[CRSF_RC_CHANNEL_COUNT] = {0U};
static volatile uint16_t g_rc_channels_us[CRSF_RC_CHANNEL_COUNT] = {
  1500U, 1500U, 1000U, 1500U,
  1500U, 1500U, 1500U, 1500U,
  1500U, 1500U, 1500U, 1500U,
  1500U, 1500U, 1500U, 1500U
};
static volatile uint32_t g_rc_frame_count = 0U;
static volatile uint32_t g_rc_last_frame_ms = 0U;
static volatile uint8_t g_rc_link_ok = 0U;

static uint8_t g_crsf_frame[CRSF_MAX_FRAME_BYTES] = {0U};
static uint8_t g_crsf_frame_index = 0U;
static uint8_t g_crsf_frame_target = 0U;
static volatile uint8_t g_crsf_rx_fifo[CRSF_RX_FIFO_SIZE] = {0U};
static volatile uint8_t g_crsf_rx_gap_fifo[CRSF_RX_FIFO_SIZE] = {0U};
static volatile uint16_t g_crsf_rx_head = 0U;
static volatile uint16_t g_crsf_rx_tail = 0U;

static void RC_SetFailsafeNeutral(void)
{
  g_rc_channels_us[0] = 1500U;
  g_rc_channels_us[1] = 1500U;
  g_rc_channels_us[2] = 1000U;
  g_rc_channels_us[3] = 1500U;
  for (uint8_t ch = 4U; ch < CRSF_RC_CHANNEL_COUNT; ch++)
  {
    g_rc_channels_us[ch] = 1500U;
  }
}
static uint8_t g_uart4_rx_byte = 0U;
static volatile uint32_t g_crsf_last_isr_byte_ms = 0U;

#define TELEMETRY_PACKET_SOF1           0xA5U
#define TELEMETRY_PACKET_SOF2           0x5AU
#define TELEMETRY_PACKET_TYPE_TELEMETRY  0x01U
#define TELEMETRY_PACKET_PAYLOAD_BYTES   65U

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */
static uint8_t MX_SPI4_Init(void);
static void Telemetry_SendPacket(int32_t pitch_cd, int32_t roll_cd, int32_t yaw_cd,
                                 int16_t gx_raw, int16_t gy_raw, int16_t gz_raw,
                                 int16_t ax_raw, int16_t ay_raw, int16_t az_raw);
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
static void IMU_CalibrateGyro(void);
static void IMU_CalibrateLevel(void);
static void IMU_UpdateEulerAngles(int16_t gx, int16_t gy, int16_t gz, int16_t ax, int16_t ay, int16_t az);
static void IMU_InitFusion(void);
static float IMU_WrapAngleDeg(float angle_deg);
static float IMU_SanitizeAngleDeg(float angle_deg, float fallback_deg);
static uint8_t IMU_TryBringup(void);
static void CRSF_ProcessUart(void);
static void CRSF_HandleFrame(const uint8_t *frame, uint8_t frame_len);
static uint8_t CRSF_CalcCrc(const uint8_t *data, uint8_t len);
static void CRSF_DecodeChannels(const uint8_t *payload);
static uint16_t CRSF_ChannelRawToUs(uint16_t raw);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
static void IMU_ApplyAlignment(float x, float y, float z, float *xo, float *yo, float *zo);
static void IMU_SetQuaternionFromEuler(float roll_deg, float pitch_deg, float yaw_deg);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void IMU_ApplyAlignment(float x, float y, float z, float *xo, float *yo, float *zo)
{
  if ((xo == NULL) || (yo == NULL) || (zo == NULL))
  {
    return;
  }

  switch (IMU_SENSOR_ALIGNMENT)
  {
    case IMU_ALIGN_CW90:
      *xo = y;
      *yo = -x;
      *zo = z;
      break;
    case IMU_ALIGN_CW180:
      *xo = -x;
      *yo = -y;
      *zo = z;
      break;
    case IMU_ALIGN_CW270:
      *xo = -y;
      *yo = x;
      *zo = z;
      break;
    case IMU_ALIGN_CW0:
    default:
      *xo = x;
      *yo = y;
      *zo = z;
      break;
  }
}

static void IMU_SetQuaternionFromEuler(float roll_deg, float pitch_deg, float yaw_deg)
{
  const float roll = roll_deg * 3.14159265f / 180.0f;
  const float pitch = pitch_deg * 3.14159265f / 180.0f;
  const float yaw = yaw_deg * 3.14159265f / 180.0f;

  const float cr = cosf(roll * 0.5f);
  const float sr = sinf(roll * 0.5f);
  const float cp = cosf(pitch * 0.5f);
  const float sp = sinf(pitch * 0.5f);
  const float cy = cosf(yaw * 0.5f);
  const float sy = sinf(yaw * 0.5f);

  g_q0 = cr * cp * cy + sr * sp * sy;
  g_q1 = sr * cp * cy - cr * sp * sy;
  g_q2 = cr * sp * cy + sr * cp * sy;
  g_q3 = cr * cp * sy - sr * sp * cy;
}

static void IMU_InitFusion(void)
{
  FusionAhrsSettings settings;
  FusionQuaternion initial_quaternion;

  FusionAhrsInitialise(&g_fusion_ahrs);

  settings = fusionAhrsDefaultSettings;
  settings.convention = FusionConventionNwu;
  settings.gain = 0.45f;
  settings.gyroscopeRange = 2000.0f;
  settings.accelerationRejection = 90.0f;
  settings.magneticRejection = 0.0f;
  settings.recoveryTriggerPeriod = 100U;
  FusionAhrsSetSettings(&g_fusion_ahrs, &settings);

  initial_quaternion.element.w = g_q0;
  initial_quaternion.element.x = g_q1;
  initial_quaternion.element.y = g_q2;
  initial_quaternion.element.z = g_q3;
  FusionAhrsSetQuaternion(&g_fusion_ahrs, initial_quaternion);

  g_fusion_ready = 1U;
}

static float IMU_WrapAngleDeg(float angle_deg)
{
  while (angle_deg > 180.0f)
  {
    angle_deg -= 360.0f;
  }
  while (angle_deg < -180.0f)
  {
    angle_deg += 360.0f;
  }
  return angle_deg;
}

static float IMU_SanitizeAngleDeg(float angle_deg, float fallback_deg)
{
  if (!isfinite(angle_deg))
  {
    return fallback_deg;
  }
  return IMU_WrapAngleDeg(angle_deg);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  uint32_t next_imu_probe_ms = 0U;
  uint32_t now_ms = 0U;
  uint32_t next_telemetry_ms = 0U;

  g_usb_ready = 1U;

  (void)IMU_TryBringup();
  next_imu_probe_ms = HAL_GetTick() + 500U;
  next_telemetry_ms = HAL_GetTick();

  if (HAL_UART_Receive_IT(&huart4, &g_uart4_rx_byte, 1U) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    int16_t gyro_x = 0;
    int16_t gyro_y = 0;
    int16_t gyro_z = 0;
    int16_t accel_x = 0;
    int16_t accel_y = 0;
    int16_t accel_z = 0;
    uint8_t sample_valid = 0U;

    CRSF_ProcessUart();
    now_ms = HAL_GetTick();
    if ((g_rc_last_frame_ms != 0U) && ((now_ms - g_rc_last_frame_ms) <= 100U))
    {
      g_rc_link_ok = 1U;
    }
    else
    {
      g_rc_link_ok = 0U;
      RC_SetFailsafeNeutral();
    }

    if ((g_imu_found == 0U) && (HAL_GetTick() >= next_imu_probe_ms))
    {
      (void)IMU_TryBringup();
      next_imu_probe_ms = HAL_GetTick() + 500U;
    }

    if (g_imu_found)
    {
      uint8_t gyro_ok = IMU_ReadRaw(&gyro_x, &gyro_y, &gyro_z);
      uint8_t accel_ok = IMU_ReadAccel(&accel_x, &accel_y, &accel_z);

      if (gyro_ok && accel_ok)
      {
        IMU_UpdateEulerAngles(gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z);
        sample_valid = 1U;

        g_pitch_cd = (int32_t)lroundf(g_pitch_fused * 100.0f);
        g_roll_cd = (int32_t)lroundf(g_roll_fused * 100.0f);
        g_yaw_cd = (int32_t)lroundf(g_yaw_fused * 100.0f);
      }
    }

    if (sample_valid == 0U)
    {
      gyro_x = 0;
      gyro_y = 0;
      gyro_z = 0;
      accel_x = 0;
      accel_y = 0;
      accel_z = 0;
    }

    now_ms = HAL_GetTick();
    if ((int32_t)(now_ms - next_telemetry_ms) >= 0)
    {
      Telemetry_SendPacket(g_pitch_cd,
                           g_roll_cd,
                           g_yaw_cd,
                           gyro_x,
                           gyro_y,
                           gyro_z,
                           accel_x,
                           accel_y,
                           accel_z);
      next_telemetry_ms = now_ms + 10U;
    }
  }
  /* USER CODE END 3 */
}

static void CRSF_ProcessUart(void)
{
  uint8_t byte = 0U;
  uint8_t had_gap = 0U;

  while (g_crsf_rx_tail != g_crsf_rx_head)
  {
    byte = g_crsf_rx_fifo[g_crsf_rx_tail];
    had_gap = g_crsf_rx_gap_fifo[g_crsf_rx_tail];
    g_crsf_rx_tail = (uint16_t)((g_crsf_rx_tail + 1U) & (CRSF_RX_FIFO_SIZE - 1U));

    if ((had_gap != 0U) && (g_crsf_frame_index != 0U))
    {
      g_crsf_frame_index = 0U;
      g_crsf_frame_target = 0U;
    }

    if (g_crsf_frame_index == 0U)
    {
      if ((byte != CRSF_SYNC_BYTE)
          && (byte != CRSF_ADDRESS_BROADCAST)
          && (byte != CRSF_ADDRESS_RADIO_TX)
          && (byte != CRSF_ADDRESS_CRSF_RX)
          && (byte != CRSF_ADDRESS_CRSF_TX))
      {
        continue;
      }

      g_crsf_frame[0] = byte;
      g_crsf_frame_index = 1U;
      g_crsf_frame_target = 0U;
      continue;
    }

    if (g_crsf_frame_index == 1U)
    {
      /* Length counts TYPE + PAYLOAD + CRC bytes. */
      if ((byte < 2U) || ((uint16_t)byte + 2U > CRSF_MAX_FRAME_BYTES))
      {
        g_crsf_frame_index = 0U;
        g_crsf_frame_target = 0U;
        continue;
      }

      g_crsf_frame[1] = byte;
      g_crsf_frame_target = (uint8_t)(byte + 2U);
      g_crsf_frame_index = 2U;
      continue;
    }

    g_crsf_frame[g_crsf_frame_index++] = byte;

    if ((g_crsf_frame_target != 0U) && (g_crsf_frame_index >= g_crsf_frame_target))
    {
      CRSF_HandleFrame(g_crsf_frame, g_crsf_frame_target);
      g_crsf_frame_index = 0U;
      g_crsf_frame_target = 0U;
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART4)
  {
    uint32_t now_ms = HAL_GetTick();
    uint8_t had_gap = 0U;
    if ((g_crsf_last_isr_byte_ms != 0U) && ((now_ms - g_crsf_last_isr_byte_ms) > CRSF_FRAME_GAP_RESET_MS))
    {
      had_gap = 1U;
    }
    g_crsf_last_isr_byte_ms = now_ms;

    uint16_t next_head = (uint16_t)((g_crsf_rx_head + 1U) & (CRSF_RX_FIFO_SIZE - 1U));
    if (next_head != g_crsf_rx_tail)
    {
      g_crsf_rx_fifo[g_crsf_rx_head] = g_uart4_rx_byte;
      g_crsf_rx_gap_fifo[g_crsf_rx_head] = had_gap;
      g_crsf_rx_head = next_head;
    }

    (void)HAL_UART_Receive_IT(&huart4, &g_uart4_rx_byte, 1U);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART4)
  {
    (void)HAL_UART_AbortReceive(huart);
    (void)HAL_UART_Receive_IT(&huart4, &g_uart4_rx_byte, 1U);
  }
}

static void CRSF_HandleFrame(const uint8_t *frame, uint8_t frame_len)
{
  uint8_t length = 0U;
  uint8_t payload_len = 0U;
  uint8_t type = 0U;
  uint8_t crc_rx = 0U;
  uint8_t crc_calc = 0U;

  if ((frame == NULL) || (frame_len < 4U))
  {
    return;
  }

  length = frame[1];
  if ((uint8_t)(length + 2U) != frame_len)
  {
    return;
  }

  type = frame[2];
  payload_len = (uint8_t)(length - 2U);

  crc_rx = frame[frame_len - 1U];
  crc_calc = CRSF_CalcCrc(&frame[2], (uint8_t)(length - 1U));

  if (crc_calc != crc_rx)
  {
    return;
  }

  if ((type == CRSF_FRAME_TYPE_RC_CHANNELS) && (payload_len == CRSF_RC_PAYLOAD_BYTES))
  {
    CRSF_DecodeChannels(&frame[3]);
    g_rc_frame_count++;
    g_rc_last_frame_ms = HAL_GetTick();
  }
}

static uint8_t CRSF_CalcCrc(const uint8_t *data, uint8_t len)
{
  uint8_t crc = 0U;

  if (data == NULL)
  {
    return 0U;
  }

  for (uint8_t i = 0U; i < len; i++)
  {
    crc ^= data[i];
    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
      if ((crc & 0x80U) != 0U)
      {
        crc = (uint8_t)((crc << 1U) ^ 0xD5U);
      }
      else
      {
        crc <<= 1U;
      }
    }
  }

  return crc;
}

static void CRSF_DecodeChannels(const uint8_t *payload)
{
  uint32_t bits = 0U;
  uint16_t raw = 0U;
  uint8_t byte_index = 0U;
  uint8_t shift = 0U;

  if (payload == NULL)
  {
    return;
  }

  for (uint8_t ch = 0U; ch < CRSF_RC_CHANNEL_COUNT; ch++)
  {
    bits = (uint32_t)ch * 11U;
    byte_index = (uint8_t)(bits / 8U);
    shift = (uint8_t)(bits % 8U);

    raw = (uint16_t)(((uint32_t)payload[byte_index]
           | ((uint32_t)payload[byte_index + 1U] << 8U)
           | ((uint32_t)payload[byte_index + 2U] << 16U)) >> shift) & 0x07FFU;

    g_rc_channels_raw[ch] = raw;
    g_rc_channels_us[ch] = CRSF_ChannelRawToUs(raw);
  }
}

static uint16_t CRSF_ChannelRawToUs(uint16_t raw)
{
  int32_t clamped = raw;

  if (clamped < 172)
  {
    clamped = 172;
  }
  if (clamped > 1811)
  {
    clamped = 1811;
  }

  /* CRSF nominal mapping: 172..1811 -> 988..2012 us */
  return (uint16_t)(988 + ((clamped - 172) * 1024) / 1639);
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 32;
  RCC_OscInitStruct.PLL.PLLN = 129;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
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
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 420000;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  huart1.Init.BaudRate = 420000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*AnalogSwitch Config */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC2, SYSCFG_SWITCH_PC2_CLOSE);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* IMU bit-banged SPI pins: PE4=CS, PE2=SCK, PE6=MOSI, PE5=MISO */
  HAL_GPIO_WritePin(GPIOE, IMU_CS_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, IMU_SCK_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, IMU_MOSI_PIN, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = IMU_CS_PIN | IMU_SCK_PIN | IMU_MOSI_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = IMU_MISO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief Send raw bytes to USB CDC device (virtual COM port)
  * @param data: payload to transmit
  * @param len: number of bytes to transmit
  * @retval None
  */
static void Serial_Write(const uint8_t *data, uint16_t len)
{
  if ((data == NULL) || (len == 0U) || (g_usb_ready == 0U))
  {
    return;
  }

  for (uint32_t retry = 0U; retry < 50U; retry++)
  {
    if (CDC_Transmit_FS((uint8_t *)data, len) == USBD_OK)
    {
      return;
    }

    HAL_Delay(1);
  }
}

static void Packet_PutU16LE(uint8_t *dst, uint16_t value)
{
  dst[0] = (uint8_t)(value & 0xFFU);
  dst[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static void Packet_PutU32LE(uint8_t *dst, uint32_t value)
{
  dst[0] = (uint8_t)(value & 0xFFU);
  dst[1] = (uint8_t)((value >> 8) & 0xFFU);
  dst[2] = (uint8_t)((value >> 16) & 0xFFU);
  dst[3] = (uint8_t)((value >> 24) & 0xFFU);
}

static void Packet_PutS16LE(uint8_t *dst, int16_t value)
{
  Packet_PutU16LE(dst, (uint16_t)value);
}

static void Packet_PutS32LE(uint8_t *dst, int32_t value)
{
  Packet_PutU32LE(dst, (uint32_t)value);
}

static void Telemetry_SendPacket(int32_t pitch_cd, int32_t roll_cd, int32_t yaw_cd,
                                 int16_t gx_raw, int16_t gy_raw, int16_t gz_raw,
                                 int16_t ax_raw, int16_t ay_raw, int16_t az_raw)
{
  uint8_t frame[4U + TELEMETRY_PACKET_PAYLOAD_BYTES + 1U];
  uint8_t checksum = 0U;
  uint8_t payload_index = 0U;

  frame[0] = TELEMETRY_PACKET_SOF1;
  frame[1] = TELEMETRY_PACKET_SOF2;
  frame[2] = TELEMETRY_PACKET_TYPE_TELEMETRY;
  frame[3] = TELEMETRY_PACKET_PAYLOAD_BYTES;

  Packet_PutU32LE(&frame[4U + payload_index], g_telemetry_sequence++);
  payload_index = (uint8_t)(payload_index + 4U);

  Packet_PutS32LE(&frame[4U + payload_index], pitch_cd);
  payload_index = (uint8_t)(payload_index + 4U);
  Packet_PutS32LE(&frame[4U + payload_index], roll_cd);
  payload_index = (uint8_t)(payload_index + 4U);
  Packet_PutS32LE(&frame[4U + payload_index], yaw_cd);
  payload_index = (uint8_t)(payload_index + 4U);

  Packet_PutS16LE(&frame[4U + payload_index], gx_raw);
  payload_index = (uint8_t)(payload_index + 2U);
  Packet_PutS16LE(&frame[4U + payload_index], gy_raw);
  payload_index = (uint8_t)(payload_index + 2U);
  Packet_PutS16LE(&frame[4U + payload_index], gz_raw);
  payload_index = (uint8_t)(payload_index + 2U);
  Packet_PutS16LE(&frame[4U + payload_index], ax_raw);
  payload_index = (uint8_t)(payload_index + 2U);
  Packet_PutS16LE(&frame[4U + payload_index], ay_raw);
  payload_index = (uint8_t)(payload_index + 2U);
  Packet_PutS16LE(&frame[4U + payload_index], az_raw);
  payload_index = (uint8_t)(payload_index + 2U);

  frame[4U + payload_index] = g_rc_link_ok;
  payload_index = (uint8_t)(payload_index + 1U);

  Packet_PutU32LE(&frame[4U + payload_index], g_rc_frame_count);
  payload_index = (uint8_t)(payload_index + 4U);

  Packet_PutU16LE(&frame[4U + payload_index], g_rc_channels_us[0]);
  payload_index = (uint8_t)(payload_index + 2U);
  Packet_PutU16LE(&frame[4U + payload_index], g_rc_channels_us[1]);
  payload_index = (uint8_t)(payload_index + 2U);
  Packet_PutU16LE(&frame[4U + payload_index], g_rc_channels_us[2]);
  payload_index = (uint8_t)(payload_index + 2U);
  Packet_PutU16LE(&frame[4U + payload_index], g_rc_channels_us[3]);
  payload_index = (uint8_t)(payload_index + 2U);
  for (uint8_t ch = 4U; ch < CRSF_RC_CHANNEL_COUNT; ch++)
  {
    Packet_PutU16LE(&frame[4U + payload_index], g_rc_channels_us[ch]);
    payload_index = (uint8_t)(payload_index + 2U);
  }

  for (uint32_t i = 2U; i < (4U + TELEMETRY_PACKET_PAYLOAD_BYTES); i++)
  {
    checksum ^= frame[i];
  }

  frame[4U + TELEMETRY_PACKET_PAYLOAD_BYTES] = checksum;

  Serial_Write(frame, (uint16_t)sizeof(frame));
}

static uint8_t MX_SPI4_Init(void)
{
  /* Bit-banged SPI uses GPIO only; pins are configured in MX_GPIO_Init. */
  g_imu_probe_error = 0U;
  HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IMU_SCK_PORT, IMU_SCK_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IMU_MOSI_PORT, IMU_MOSI_PIN, GPIO_PIN_RESET);

  return 1U;
}

static uint8_t IMU_TryBringup(void)
{
  if (MX_SPI4_Init() == 0U)
  {
    g_imu_found = 0U;
    return 0U;
  }

  g_imu_found = IMU_Probe();
  if (g_imu_found == 0U)
  {
    return 0U;
  }

  if (IMU_Config() == 0U)
  {
    g_imu_found = 0U;
    return 0U;
  }

  HAL_Delay(100);
  IMU_CalibrateGyro();
  HAL_Delay(100);
  IMU_CalibrateLevel();
  IMU_InitFusion();
  HAL_Delay(500);

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
  uint8_t whoami_verify = 0U;

  if (IMU_ReadWhoAmI(IMU_WHOAMI_REG_MPU, &whoami) != 0U)
  {
    if ((whoami == 0x68U) || (whoami == 0x70U) || (whoami == 0x71U) ||
        (whoami == 0x42U) || (whoami == 0x47U))
    {
      if ((IMU_ReadReg(IMU_WHOAMI_REG_MPU, &whoami_verify) == 0U) || (whoami_verify != whoami))
      {
        g_imu_probe_error = 0xE304U; /* unstable WHO_AM_I */
        return 0U;
      }

      g_imu_bus_id = 4U; /* bus marker: SPI4 */
      g_imu_whoami = whoami;
      return 1U;
    }
  }

  if (IMU_ReadWhoAmI(IMU_WHOAMI_REG_BMI270, &whoami) != 0U)
  {
    if (whoami == 0x24U)
    {
      g_imu_probe_error = 0xE320U; /* BMI270 detected: current config path supports ICM-class only */
      return 0U;
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
  IMU_SpiDelay();  /* Ensure CS is settled before first transfer */
  
  (void)IMU_SpiTransfer((uint8_t)(startReg | 0x80U));
  IMU_SpiDelay();  /* Allow IMU to respond */

  for (uint8_t i = 0U; i < len; i++)
  {
    data[i] = IMU_SpiTransfer(0x00U);
    IMU_SpiDelay();  /* Brief delay between bytes */
  }

  HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);
  IMU_SpiDelay();  /* Ensure CS is settled before release */
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

/**
 * @brief Calibrate gyro by averaging 100 readings while board is stationary
 */
static void IMU_CalibrateGyro(void)
{
  int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
  int16_t gx, gy, gz;
  
  for (int i = 0; i < 100; i++)
  {
    if (IMU_ReadRaw(&gx, &gy, &gz))
    {
      sum_gx += gx;
      sum_gy += gy;
      sum_gz += gz;
    }
    HAL_Delay(10);
  }
  
  g_gyro_bias_x = sum_gx / 100;
  g_gyro_bias_y = sum_gy / 100;
  g_gyro_bias_z = sum_gz / 100;
  g_gyro_calibrated = 1U;
}

/**
 * @brief Calibrate level position (initialize pitch/roll from accel)
 * Reads accel for ~1 second on level surface to establish baseline angles
 */
static void IMU_CalibrateLevel(void)
{
  float sum_pitch = 0.0f, sum_roll = 0.0f;
  int16_t ax, ay, az;
  int num_samples = 100;
  
  for (int i = 0; i < num_samples; i++)
  {
    if (IMU_ReadAccel(&ax, &ay, &az))
    {
      /* Convert accel to g units (±16g range) */
      float ax_g = (float)ax / IMU_ACCEL_LSB_PER_G;
      float ay_g = (float)ay / IMU_ACCEL_LSB_PER_G;
      float az_g = (float)az / IMU_ACCEL_LSB_PER_G;
      float ax_aligned = 0.0f;
      float ay_aligned = 0.0f;
      float az_aligned = 0.0f;

      IMU_ApplyAlignment(ax_g, ay_g, az_g, &ax_aligned, &ay_aligned, &az_aligned);
      
      /* Tilt-stable accel angles: minimizes roll/pitch cross-coupling near ±90 deg pitch. */
      float accel_pitch = atan2f(-ax_aligned, sqrtf((ay_aligned * ay_aligned) + (az_aligned * az_aligned))) * 180.0f / 3.14159265f;
      float accel_roll = -atan2f(ay_aligned, az_aligned) * 180.0f / 3.14159265f;
      
      sum_pitch += accel_pitch;
      sum_roll += accel_roll;
    }
    HAL_Delay(10);
  }
  
  /* Initialize fused angles to accel baseline */
  g_pitch_fused = sum_pitch / num_samples;
  g_roll_fused = sum_roll / num_samples;
  g_yaw_fused = 0.0f;  /* Yaw always starts at 0 (no absolute reference) */
  
  /* If very close to zero (within ±2 degrees), snap to exact zero to eliminate noise */
  if (fabsf(g_pitch_fused) < 2.0f) g_pitch_fused = 0.0f;
  if (fabsf(g_roll_fused) < 2.0f) g_roll_fused = 0.0f;

  IMU_SetQuaternionFromEuler(g_roll_fused, g_pitch_fused, g_yaw_fused);
}

/**
 * @brief Update Euler angles using complementary filter
 * Combines accelerometer (absolute reference) with gyro (fast response)
 */
static void IMU_UpdateEulerAngles(int16_t gx, int16_t gy, int16_t gz, int16_t ax, int16_t ay, int16_t az)
{
  static uint32_t s_last_update_ms = 0U;
  uint32_t now_ms = HAL_GetTick();
  float dt = 0.01f;

  if (s_last_update_ms != 0U)
  {
    uint32_t delta_ms = now_ms - s_last_update_ms;
    if (delta_ms == 0U)
    {
      dt = 0.001f;
    }
    else
    {
      dt = (float)delta_ms * 0.001f;
      if (dt < 0.001f)
      {
        dt = 0.001f;
      }
      else if (dt > 0.05f)
      {
        dt = 0.05f;
      }
    }
  }
  s_last_update_ms = now_ms;

  /* Apply gyro bias correction */
  int16_t gx_corrected = gx - (int16_t)g_gyro_bias_x;
  int16_t gy_corrected = gy - (int16_t)g_gyro_bias_y;
  int16_t gz_corrected = gz - (int16_t)g_gyro_bias_z;
  
  /* Convert gyro to degrees/sec (2000 dps range) */
  float gx_dps = (float)gx_corrected * IMU_GYRO_DPS_PER_LSB;
  float gy_dps = (float)gy_corrected * IMU_GYRO_DPS_PER_LSB;
  float gz_dps = (float)gz_corrected * IMU_GYRO_DPS_PER_LSB;
  float gx_aligned = 0.0f;
  float gy_aligned = 0.0f;
  float gz_aligned = 0.0f;

  IMU_ApplyAlignment(gx_dps, gy_dps, gz_dps, &gx_aligned, &gy_aligned, &gz_aligned);
  
  /* Convert accel to g units (±16g range) */
  float ax_g = (float)ax / IMU_ACCEL_LSB_PER_G;
  float ay_g = (float)ay / IMU_ACCEL_LSB_PER_G;
  float az_g = (float)az / IMU_ACCEL_LSB_PER_G;
  float ax_aligned = 0.0f;
  float ay_aligned = 0.0f;
  float az_aligned = 0.0f;

  IMU_ApplyAlignment(ax_g, ay_g, az_g, &ax_aligned, &ay_aligned, &az_aligned);

  if (g_fusion_ready == 0U)
  {
    IMU_InitFusion();
  }

  {
    FusionVector gyro_vector;
    FusionVector accel_vector;
    FusionQuaternion q;
    FusionEuler euler;
    float roll_next;
    float pitch_next;
    float yaw_next;

    gyro_vector.axis.x = gx_aligned;
    gyro_vector.axis.y = gy_aligned;
    gyro_vector.axis.z = gz_aligned;

    accel_vector.axis.x = ax_aligned;
    accel_vector.axis.y = ay_aligned;
    accel_vector.axis.z = az_aligned;

    FusionAhrsUpdateNoMagnetometer(&g_fusion_ahrs, gyro_vector, accel_vector, dt);

    q = FusionAhrsGetQuaternion(&g_fusion_ahrs);

    if ((!isfinite(q.element.w)) || (!isfinite(q.element.x)) || (!isfinite(q.element.y)) || (!isfinite(q.element.z)))
    {
      /* Recover from rare AHRS numerical fault without corrupting telemetry stream. */
      IMU_InitFusion();
      q = FusionAhrsGetQuaternion(&g_fusion_ahrs);
    }

    g_q0 = q.element.w;
    g_q1 = q.element.x;
    g_q2 = q.element.y;
    g_q3 = q.element.z;

    euler = FusionQuaternionToEuler(q);
    roll_next = IMU_SanitizeAngleDeg(-euler.angle.roll, g_roll_fused);
    pitch_next = IMU_SanitizeAngleDeg(euler.angle.pitch, g_pitch_fused);
    yaw_next = IMU_SanitizeAngleDeg(euler.angle.yaw, g_yaw_fused);

    g_roll_fused = roll_next;
    g_pitch_fused = pitch_next;
    g_yaw_fused = yaw_next;
  }

  g_pitch_fused = IMU_WrapAngleDeg(g_pitch_fused);
  g_roll_fused = IMU_WrapAngleDeg(g_roll_fused);
  g_yaw_fused = IMU_WrapAngleDeg(g_yaw_fused);
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

  /* BMI270 uses big-endian: high byte first, then low byte */
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

  /* BMI270 uses big-endian: high byte first, then low byte */
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
    /* CPOL=1, CPHA=1: change on falling edge, sample on rising edge. */
    HAL_GPIO_WritePin(IMU_MOSI_PORT,
                      IMU_MOSI_PIN,
                      ((tx & 0x80U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    tx <<= 1;

    HAL_GPIO_WritePin(IMU_SCK_PORT, IMU_SCK_PIN, GPIO_PIN_RESET);
    IMU_SpiDelay();

    HAL_GPIO_WritePin(IMU_SCK_PORT, IMU_SCK_PIN, GPIO_PIN_SET);
    IMU_SpiDelay();

    rx <<= 1;
    if (HAL_GPIO_ReadPin(IMU_MISO_PORT, IMU_MISO_PIN) == GPIO_PIN_SET)
    {
      rx |= 1U;
    }
  }

  return rx;
}

static void IMU_SpiDelay(void)
{
  /* Provide ~1 microsecond delay (10 NOPs was too fast at 120 MHz) */
  for (volatile uint32_t i = 0U; i < 1000U; i++)
  {
    __NOP();
  }
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
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
