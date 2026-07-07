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
#include <string.h>
#include "usbd_cdc_if.h"
#include "Fusion.h"
#include "display_filter.h"
#include "rate_controller.h"
#include "telemetry_link.h"

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

#define BATTERY_ADC_PORT                GPIOC
#define BATTERY_ADC_PIN                 GPIO_PIN_0
#define BATTERY_ADC_CHANNEL             ADC_CHANNEL_10
#define BATTERY_VREF_MV                 3300U
#define BATTERY_ADC_MAX_COUNTS          4095U
/* Calibrated from measured 11.34V vs previously reported 14.69V. */
#define BATTERY_DIVIDER_NUM             849U
#define BATTERY_DIVIDER_DEN             100U

#define IMU_ALIGN_CW0           0U
#define IMU_ALIGN_CW90          1U
#define IMU_ALIGN_CW180         2U
#define IMU_ALIGN_CW270         3U
/* Temporary diagnostic mode: keep only USB bring-up to isolate enumeration issues. */
#define USB_ENUM_DIAGNOSTIC     0U
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

#define MOTOR_COUNT                     4U
#define MOTOR_PWM_MIN_US                1000U
#define MOTOR_PWM_MAX_US                2000U
#define MOTOR_COMMAND_MIN_US            1100U
#define MOTOR_ARM_SWITCH_US             1600U
#define MOTOR_THROTTLE_MIN_US           1000U
#define MOTOR_THROTTLE_MAX_US           2000U
#define MOTOR_MODE_SWITCH_US            1600U
#define MOTOR_MODE_SWITCH_CHANNEL_FIRST 5U   /* CH6 */
#define MOTOR_MODE_SWITCH_CHANNEL_LAST  11U  /* CH12 */
#define MOTOR_OUTPUT_MODE_SIMULATION    0U
#define MOTOR_OUTPUT_MODE_REAL_THROTTLE 1U
#define MOTOR_FIXED_TEST_ENABLE         0U
#define MOTOR_FIXED_TEST_ENABLE_CHANNEL 7U   /* CH8 */
#define MOTOR_FIXED_TEST_SELECT_CHANNEL 8U   /* CH9 */
#define MOTOR_FIXED_TEST_ENABLE_US      1600U
#define MOTOR_FIXED_TEST_US             1300U
#define MOTOR_FORCE_ALL_TEST_ENABLE     0U
#define MOTOR_FORCE_ALL_TEST_US         1600U
#define MOTOR_PWM_SELF_TEST_ENABLE      0U
#define MOTOR_PWM_SELF_TEST_US          1600U
#define MOTOR_INIT_AT_BOOT              1U
#define MOTOR_HW_OUTPUT_ENABLE          1U
#define MOTOR_BACKEND_PWM               0U
#define MOTOR_BACKEND_DSHOT             1U
#define MOTOR_BACKEND                   MOTOR_BACKEND_PWM
#define MOTOR_PWM_RATE_HZ               50U
#define MOTOR_TIMER_TICK_HZ             1000000U
#define MOTOR_SPIN_FLOOR_US             1070U
#define MOTOR_DSHOT_MIN_ACTIVE_VALUE    48U
/* Diagnostic: also drive legacy PA0-PA3 TIM2 outputs while using INAV S1-S4 map. */
#define MOTOR_OUTPUT_DUAL_MAP_DIAG      0U
#define RATE_CMD_DEADBAND               0.05f
#define RATE_CMD_ROLL_MAX_DPS           120.0f
#define RATE_CMD_PITCH_MAX_DPS          120.0f
#define RATE_CMD_YAW_MAX_DPS            90.0f
#define RATE_CMD_ROLL_SIGN              (-1.0f)
#define RATE_CMD_PITCH_SIGN             (-1.0f)

#define MOTOR1_GPIO_PIN                 GPIO_PIN_0   /* S1: TIM3_CH3, M1 aft-right */
#define MOTOR2_GPIO_PIN                 GPIO_PIN_1   /* S2: TIM3_CH4, M2 forward-right */
#define MOTOR3_GPIO_PIN                 GPIO_PIN_3   /* S3: TIM2_CH2, M3 aft-left */
#define MOTOR4_GPIO_PIN                 GPIO_PIN_10  /* S4: TIM2_CH3, M4 forward-left */
#define MOTOR_DSHOT_GPIO_MASK           (MOTOR1_GPIO_PIN | MOTOR2_GPIO_PIN | MOTOR3_GPIO_PIN | MOTOR4_GPIO_PIN)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart6;
UART_HandleTypeDef huart1;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

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
static float g_gyro_roll_rate_dps = 0.0f;
static float g_gyro_pitch_rate_dps = 0.0f;
static float g_gyro_yaw_rate_dps = 0.0f;
static FusionAhrs g_fusion_ahrs;
static uint8_t g_fusion_ready = 0U;

static uint8_t g_imu_spi_mode = 3U; /* 3=mode3, 0=mode0 */
static uint8_t g_usb_ready = 0U;
static uint32_t g_telemetry_sequence = 0U;

static uint8_t g_bmp280_found = 0U;
static uint8_t g_bmp280_addr = 0x76U;
static uint32_t g_baro_pressure_pa = 101325U;
static int32_t g_baro_altitude_cm = 0;
static float g_baro_reference_pressure_pa = 101325.0f;
static int32_t g_baro_temp_raw = 0;
static int32_t g_baro_press_raw = 0;
static int32_t g_baro_t_fine = 0;

static uint16_t g_bmp280_dig_t1 = 0U;
static int16_t g_bmp280_dig_t2 = 0;
static int16_t g_bmp280_dig_t3 = 0;
static uint16_t g_bmp280_dig_p1 = 0U;
static int16_t g_bmp280_dig_p2 = 0;
static int16_t g_bmp280_dig_p3 = 0;
static int16_t g_bmp280_dig_p4 = 0;
static int16_t g_bmp280_dig_p5 = 0;
static int16_t g_bmp280_dig_p6 = 0;
static int16_t g_bmp280_dig_p7 = 0;
static int16_t g_bmp280_dig_p8 = 0;
static int16_t g_bmp280_dig_p9 = 0;

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
static uint8_t g_motors_armed = 0U;
static uint8_t g_motor_output_mode = MOTOR_OUTPUT_MODE_SIMULATION;
static uint8_t g_motor_hw_ready = 0U;
static uint16_t g_motor_cmd_us[MOTOR_COUNT] = {
  MOTOR_PWM_MIN_US,
  MOTOR_PWM_MIN_US,
  MOTOR_PWM_MIN_US,
  MOTOR_PWM_MIN_US
};
static uint8_t g_motor_gui_test_active = 0U;
static uint8_t g_motor_gui_test_index = 0U;
static uint16_t g_motor_gui_test_us = MOTOR_COMMAND_MIN_US;
static uint32_t g_motor_gui_test_expire_ms = 0U;
static uint8_t g_usb_cmd_frame[16] = {0U};
static uint8_t g_usb_cmd_frame_index = 0U;
static uint8_t g_usb_cmd_frame_target = 0U;
static char g_usb_cmd_line[32] = {0};
static uint8_t g_usb_cmd_line_index = 0U;
static volatile float g_display_filter_cutoff_hz = 5.0f;
static int16_t g_display_filter_gx = 0;
static int16_t g_display_filter_gy = 0;
static int16_t g_display_filter_gz = 0;
static uint16_t g_battery_voltage_mv = 0U;
static uint8_t g_battery_adc_ready = 0U;
static uint8_t g_battery_adc_channel_index = 10U;
static uint8_t g_battery_adc_candidate_pos = 0U;
static uint8_t g_battery_adc_zero_streak = 0U;
static uint16_t g_battery_adc_raw_last = 0U;
static float g_desired_roll_rate_dps = 0.0f;
static float g_desired_pitch_rate_dps = 0.0f;
static float g_desired_yaw_rate_dps = 0.0f;
static float g_roll_rate_error_dps = 0.0f;
static float g_pitch_rate_error_dps = 0.0f;
static float g_yaw_rate_error_dps = 0.0f;
static float g_roll_pid_output = 0.0f;
static float g_pitch_pid_output = 0.0f;
static float g_yaw_pid_output = 0.0f;
static RateController g_rate_controller;

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
#define TELEMETRY_PACKET_TYPE_MOTOR_TEST 0x10U
#define TELEMETRY_PACKET_TYPE_DISPLAY_FILTER 0x11U
#define TELEMETRY_PACKET_PAYLOAD_BYTES_V8    122U
#define MOTOR_TEST_PAYLOAD_BYTES         3U
#define DISPLAY_FILTER_PAYLOAD_BYTES     2U
#define MOTOR_TEST_TIMEOUT_MS            250U

#define BMP280_I2C_TIMEOUT_MS           20U
#define BMP280_REG_ID                   0xD0U
#define BMP280_REG_RESET                0xE0U
#define BMP280_REG_STATUS               0xF3U
#define BMP280_REG_CTRL_MEAS            0xF4U
#define BMP280_REG_CONFIG               0xF5U
#define BMP280_REG_PRESS_MSB            0xF7U
#define BMP280_REG_CALIB_START          0x88U
#define BMP280_CHIP_ID                  0x58U

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static uint8_t MX_ADC1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */
static uint8_t MX_SPI4_Init(void);
static void Telemetry_SendPacket(int32_t pitch_cd, int32_t roll_cd, int32_t yaw_cd,
                                 int16_t gx_raw, int16_t gy_raw, int16_t gz_raw,
                                 int16_t gx_filtered, int16_t gy_filtered, int16_t gz_filtered,
                                 int16_t ax_raw, int16_t ay_raw, int16_t az_raw,
                                 uint32_t pressure_pa, int32_t altitude_cm);
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
static uint8_t BMP280_Init(void);
static uint8_t BMP280_Update(void);
static void Battery_Update(void);
static void Battery_SelectAdcChannel(uint8_t channel_index);
static uint32_t Battery_ReadChannelRaw(uint8_t channel_index, uint8_t *ok);
static HAL_StatusTypeDef BMP280_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len);
static HAL_StatusTypeDef BMP280_WriteReg(uint8_t reg, uint8_t value);
static float BMP280_PressureToAltitudeCm(float pressure_pa, float reference_pressure_pa);
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
static void MotorControl_UpdateVirtual(void);
static void MotorControl_Disarm(void);
static void MotorGui_ProcessUsbCommands(void);
static void MotorGui_HandleMotorTest(uint8_t motor_index, uint16_t motor_us);
static void MotorGui_HandleDisplayFilter(uint16_t cutoff_hz);
static void MotorGui_ProcessAsciiByte(uint8_t byte);
static void MotorGui_TryParseAsciiLine(const char *line);
static uint8_t MX_TIM2_Init_MotorPwm(void);
static void MotorOutput_InitAndStart(void);
static void MotorOutput_BootHold(uint32_t duration_ms);
static void MotorOutput_WriteMicroseconds(const uint16_t *motor_us);
static float MotorControl_ClampF(float value, float min_value, float max_value);
static float MotorControl_ApplyDeadband(float value, float deadband);
static float MotorControl_ChannelToBiPolar(uint16_t us);
static float MotorControl_ChannelToThrottle(uint16_t us);
static uint8_t Packet_CalcXorChecksum(uint8_t packet_type, uint8_t payload_len, const uint8_t *payload);
static uint16_t Parse_DecU16(const char *text, uint16_t *consumed);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static float MotorControl_ClampF(float value, float min_value, float max_value)
{
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return value;
}

static float MotorControl_ApplyDeadband(float value, float deadband)
{
  float magnitude = value;

  if (magnitude < 0.0f)
  {
    magnitude = -magnitude;
  }

  if (magnitude <= deadband)
  {
    return 0.0f;
  }

  if (value > 0.0f)
  {
    return (value - deadband) / (1.0f - deadband);
  }

  return (value + deadband) / (1.0f - deadband);
}

static float MotorControl_ChannelToBiPolar(uint16_t us)
{
  float centered = ((float)us - 1500.0f) / 500.0f;
  return MotorControl_ClampF(centered, -1.0f, 1.0f);
}

static float MotorControl_ChannelToThrottle(uint16_t us)
{
  float normalized = ((float)us - (float)MOTOR_THROTTLE_MIN_US)
                   / (float)(MOTOR_THROTTLE_MAX_US - MOTOR_THROTTLE_MIN_US);
  return MotorControl_ClampF(normalized, 0.0f, 1.0f);
}

static uint8_t Packet_CalcXorChecksum(uint8_t packet_type, uint8_t payload_len, const uint8_t *payload)
{
  uint8_t checksum = (uint8_t)(packet_type ^ payload_len);

  if (payload != NULL)
  {
    for (uint8_t i = 0U; i < payload_len; i++)
    {
      checksum ^= payload[i];
    }
  }

  return checksum;
}

static uint16_t Parse_DecU16(const char *text, uint16_t *consumed)
{
  uint32_t value = 0U;
  uint16_t i = 0U;

  if (consumed == NULL)
  {
    return 0U;
  }

  while ((text != NULL) && (text[i] >= '0') && (text[i] <= '9'))
  {
    value = (value * 10U) + (uint32_t)(text[i] - '0');
    if (value > 65535U)
    {
      value = 65535U;
    }
    i++;
  }

  *consumed = i;
  return (uint16_t)value;
}

static void MotorGui_HandleMotorTest(uint8_t motor_index, uint16_t motor_us)
{
  if ((motor_index < 1U) || (motor_index > MOTOR_COUNT))
  {
    g_motor_gui_test_active = 0U;
    g_motor_gui_test_us = MOTOR_COMMAND_MIN_US;
    g_motor_gui_test_expire_ms = 0U;
    return;
  }

  g_motor_gui_test_index = (uint8_t)(motor_index - 1U);
  g_motor_gui_test_us = (uint16_t)MotorControl_ClampF((float)motor_us, (float)MOTOR_COMMAND_MIN_US, (float)MOTOR_PWM_MAX_US);
  g_motor_gui_test_expire_ms = HAL_GetTick() + MOTOR_TEST_TIMEOUT_MS;
  g_motor_gui_test_active = 1U;
}

static void MotorGui_HandleDisplayFilter(uint16_t cutoff_hz)
{
  float clamped_hz = MotorControl_ClampF((float)cutoff_hz, 0.0f, 10.0f);

  g_display_filter_cutoff_hz = clamped_hz;
  DisplayFilter_SetCutoffHz(clamped_hz);
}

static void MotorGui_TryParseAsciiLine(const char *line)
{
  uint16_t consumed = 0U;
  uint16_t motor_u16 = 0U;
  uint16_t us_u16 = 0U;
  uint16_t pos = 0U;

  if (line == NULL)
  {
    return;
  }

  if (strncmp(line, "MTEST ", 6U) == 0)
  {
    pos = 6U;
  }
  else if (strncmp(line, "MTEST,", 6U) == 0)
  {
    pos = 6U;
  }
  else if (strncmp(line, "DFLT ", 5U) == 0)
  {
    pos = 5U;
  }
  else if (strncmp(line, "DFLT,", 5U) == 0)
  {
    pos = 5U;
  }
  else
  {
    return;
  }

  while (line[pos] == ' ')
  {
    pos++;
  }

  motor_u16 = Parse_DecU16(&line[pos], &consumed);
  if (consumed == 0U)
  {
    return;
  }
  pos = (uint16_t)(pos + consumed);

  while ((line[pos] == ' ') || (line[pos] == ','))
  {
    pos++;
  }

  us_u16 = Parse_DecU16(&line[pos], &consumed);
  if (consumed == 0U)
  {
    return;
  }

  if ((line[0] == 'D') && (line[1] == 'F') && (line[2] == 'L') && (line[3] == 'T'))
  {
    MotorGui_HandleDisplayFilter(us_u16);
    return;
  }

  MotorGui_HandleMotorTest((uint8_t)motor_u16, us_u16);
}

static void MotorGui_ProcessAsciiByte(uint8_t byte)
{
  if ((byte == '\r') || (byte == '\n'))
  {
    if (g_usb_cmd_line_index != 0U)
    {
      g_usb_cmd_line[g_usb_cmd_line_index] = '\0';
      MotorGui_TryParseAsciiLine(g_usb_cmd_line);
      g_usb_cmd_line_index = 0U;
    }
    return;
  }

  if ((byte >= 32U) && (byte <= 126U))
  {
    if (g_usb_cmd_line_index < (uint8_t)(sizeof(g_usb_cmd_line) - 1U))
    {
      g_usb_cmd_line[g_usb_cmd_line_index++] = (char)byte;
    }
    else
    {
      g_usb_cmd_line_index = 0U;
    }
    return;
  }

  g_usb_cmd_line_index = 0U;
}

static void MotorGui_ProcessUsbCommands(void)
{
  uint8_t byte = 0U;

  while (CDC_ReadByte_FS(&byte) != 0U)
  {
    MotorGui_ProcessAsciiByte(byte);

    if (g_usb_cmd_frame_index == 0U)
    {
      if (byte == TELEMETRY_PACKET_SOF1)
      {
        g_usb_cmd_frame[0] = byte;
        g_usb_cmd_frame_index = 1U;
      }
      continue;
    }

    if (g_usb_cmd_frame_index == 1U)
    {
      if (byte == TELEMETRY_PACKET_SOF2)
      {
        g_usb_cmd_frame[1] = byte;
        g_usb_cmd_frame_index = 2U;
      }
      else if (byte == TELEMETRY_PACKET_SOF1)
      {
        g_usb_cmd_frame[0] = byte;
        g_usb_cmd_frame_index = 1U;
      }
      else
      {
        g_usb_cmd_frame_index = 0U;
      }
      continue;
    }

    if (g_usb_cmd_frame_index == 2U)
    {
      g_usb_cmd_frame[2] = byte;
      g_usb_cmd_frame_index = 3U;
      continue;
    }

    if (g_usb_cmd_frame_index == 3U)
    {
      if (byte > 8U)
      {
        g_usb_cmd_frame_index = 0U;
        continue;
      }

      g_usb_cmd_frame[3] = byte;
      g_usb_cmd_frame_target = (uint8_t)(4U + byte + 1U);
      if (g_usb_cmd_frame_target > (uint8_t)sizeof(g_usb_cmd_frame))
      {
        g_usb_cmd_frame_index = 0U;
        continue;
      }

      g_usb_cmd_frame_index = 4U;
      continue;
    }

    g_usb_cmd_frame[g_usb_cmd_frame_index++] = byte;

    if ((g_usb_cmd_frame_target != 0U) && (g_usb_cmd_frame_index >= g_usb_cmd_frame_target))
    {
      const uint8_t packet_type = g_usb_cmd_frame[2];
      const uint8_t payload_len = g_usb_cmd_frame[3];
      const uint8_t *payload = &g_usb_cmd_frame[4];
      const uint8_t checksum_rx = g_usb_cmd_frame[g_usb_cmd_frame_target - 1U];
      const uint8_t checksum_calc = Packet_CalcXorChecksum(packet_type, payload_len, payload);

      if (checksum_rx == checksum_calc)
      {
        if ((packet_type == TELEMETRY_PACKET_TYPE_MOTOR_TEST) && (payload_len == MOTOR_TEST_PAYLOAD_BYTES))
        {
          const uint8_t motor_index = payload[0];
          const uint16_t motor_us = (uint16_t)payload[1] | ((uint16_t)payload[2] << 8U);
          MotorGui_HandleMotorTest(motor_index, motor_us);
        }
        else if ((packet_type == TELEMETRY_PACKET_TYPE_DISPLAY_FILTER) && (payload_len == DISPLAY_FILTER_PAYLOAD_BYTES))
        {
          const uint16_t cutoff_hz = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8U);
          MotorGui_HandleDisplayFilter(cutoff_hz);
        }
      }

      g_usb_cmd_frame_index = 0U;
      g_usb_cmd_frame_target = 0U;
    }
  }
}

static void MotorControl_Disarm(void)
{
  g_motors_armed = 0U;
  for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
  {
    g_motor_cmd_us[i] = MOTOR_PWM_MIN_US;
  }

  MotorOutput_WriteMicroseconds(g_motor_cmd_us);
}

static uint8_t MX_TIM2_Init_MotorPwm(void)
{
#if (MOTOR_HW_OUTPUT_ENABLE != 0U)
#if (MOTOR_BACKEND == MOTOR_BACKEND_DSHOT)
  g_motor_dshot_ready = dshot_init();
  return g_motor_dshot_ready;
#else
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  uint32_t tim_clk_hz = HAL_RCC_GetPCLK1Freq();

  if (tim_clk_hz < MOTOR_TIMER_TICK_HZ)
  {
    return 0U;
  }

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();

  GPIO_InitStruct.Pin = MOTOR1_GPIO_PIN | MOTOR2_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = MOTOR3_GPIO_PIN | MOTOR4_GPIO_PIN;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

#if (MOTOR_OUTPUT_DUAL_MAP_DIAG != 0U)
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = (uint32_t)(tim_clk_hz / MOTOR_TIMER_TICK_HZ) - 1U;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = (MOTOR_TIMER_TICK_HZ / MOTOR_PWM_RATE_HZ) - 1U;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    return 0U;
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = MOTOR_PWM_MIN_US;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    return 0U;
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    return 0U;
  }
#if (MOTOR_OUTPUT_DUAL_MAP_DIAG != 0U)
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    return 0U;
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    return 0U;
  }
#endif

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = (uint32_t)(tim_clk_hz / MOTOR_TIMER_TICK_HZ) - 1U;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = (MOTOR_TIMER_TICK_HZ / MOTOR_PWM_RATE_HZ) - 1U;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    return 0U;
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    return 0U;
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    return 0U;
  }

  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK)
  {
    return 0U;
  }
  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK)
  {
    return 0U;
  }
#if (MOTOR_OUTPUT_DUAL_MAP_DIAG != 0U)
  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
  {
    return 0U;
  }
  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4) != HAL_OK)
  {
    return 0U;
  }
#endif
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK)
  {
    return 0U;
  }
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK)
  {
    return 0U;
  }

  return 1U;
#endif
#else
  return 0U;
#endif
}

static void MotorOutput_InitAndStart(void)
{
  g_motor_hw_ready = MX_TIM2_Init_MotorPwm();
  MotorOutput_WriteMicroseconds(g_motor_cmd_us);
}

static void MotorOutput_BootHold(uint32_t duration_ms)
{
  const uint32_t start_ms = HAL_GetTick();

  while ((HAL_GetTick() - start_ms) < duration_ms)
  {
    MotorOutput_WriteMicroseconds(g_motor_cmd_us);
    HAL_Delay(2);
  }
}

static void MotorOutput_WriteMicroseconds(const uint16_t *motor_us)
{
#if (MOTOR_HW_OUTPUT_ENABLE != 0U)
#if (MOTOR_BACKEND == MOTOR_BACKEND_DSHOT)
  if ((g_motor_hw_ready == 0U) || (motor_us == NULL))
  {
    return;
  }

  /* M1-only DMA DShot bring-up on S1/PB0 (TIM3_CH3). */
  {
    uint16_t min_us = (g_motors_armed != 0U) ? MOTOR_COMMAND_MIN_US : MOTOR_PWM_MIN_US;
    uint16_t command_us = (uint16_t)MotorControl_ClampF((float)motor_us[0], (float)min_us, (float)MOTOR_PWM_MAX_US);
    uint16_t dshot_value = MotorDshot_UsToValue(command_us);
    if ((g_motors_armed != 0U) && (dshot_value == 0U))
    {
      dshot_value = MOTOR_DSHOT_MIN_ACTIVE_VALUE;
    }
    dshot_write_motor(0U, dshot_value);
  }
#else
  uint16_t m1 = MOTOR_PWM_MIN_US;
  uint16_t m2 = MOTOR_PWM_MIN_US;
  uint16_t m3 = MOTOR_PWM_MIN_US;
  uint16_t m4 = MOTOR_PWM_MIN_US;
  uint16_t min_us = (g_motors_armed != 0U) ? MOTOR_COMMAND_MIN_US : MOTOR_PWM_MIN_US;

  if ((g_motor_hw_ready == 0U) || (motor_us == NULL))
  {
    return;
  }

  m1 = (uint16_t)MotorControl_ClampF((float)motor_us[0], (float)min_us, (float)MOTOR_PWM_MAX_US);
  m2 = (uint16_t)MotorControl_ClampF((float)motor_us[1], (float)min_us, (float)MOTOR_PWM_MAX_US);
  m3 = (uint16_t)MotorControl_ClampF((float)motor_us[2], (float)min_us, (float)MOTOR_PWM_MAX_US);
  m4 = (uint16_t)MotorControl_ClampF((float)motor_us[3], (float)min_us, (float)MOTOR_PWM_MAX_US);

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, m1);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, m2);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, m3);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, m4);
#if (MOTOR_OUTPUT_DUAL_MAP_DIAG != 0U)
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, m1);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, m4);
#endif
#endif
#else
  UNUSED(motor_us);
#endif
}

/* Step 1 motor-controller scaffold: software mixer only, no PWM output yet. */
static void MotorControl_UpdateVirtual(void)
{
  const uint8_t arm_switch_on = (g_rc_channels_us[4] >= MOTOR_ARM_SWITCH_US) ? 1U : 0U;
  const uint8_t fixed_test_on = (g_rc_channels_us[MOTOR_FIXED_TEST_ENABLE_CHANNEL] >= MOTOR_FIXED_TEST_ENABLE_US) ? 1U : 0U;
  const uint32_t now_ms = HAL_GetTick();
  uint8_t real_mode_on = 0U;
  const float throttle = MotorControl_ChannelToThrottle(g_rc_channels_us[2]);
  const float roll = RATE_CMD_ROLL_SIGN * MotorControl_ChannelToBiPolar(g_rc_channels_us[0]);
  const float pitch = RATE_CMD_PITCH_SIGN * MotorControl_ChannelToBiPolar(g_rc_channels_us[1]);
  const float yaw = MotorControl_ChannelToBiPolar(g_rc_channels_us[3]);
  const float roll_rate_cmd = MotorControl_ApplyDeadband(roll, RATE_CMD_DEADBAND) * RATE_CMD_ROLL_MAX_DPS;
  const float pitch_rate_cmd = MotorControl_ApplyDeadband(pitch, RATE_CMD_DEADBAND) * RATE_CMD_PITCH_MAX_DPS;
  const float yaw_rate_cmd = MotorControl_ApplyDeadband(yaw, RATE_CMD_DEADBAND) * RATE_CMD_YAW_MAX_DPS;

  g_desired_roll_rate_dps = roll_rate_cmd;
  g_desired_pitch_rate_dps = pitch_rate_cmd;
  g_desired_yaw_rate_dps = yaw_rate_cmd;

  for (uint8_t mode_ch = MOTOR_MODE_SWITCH_CHANNEL_FIRST; mode_ch <= MOTOR_MODE_SWITCH_CHANNEL_LAST; mode_ch++)
  {
    if (g_rc_channels_us[mode_ch] >= MOTOR_MODE_SWITCH_US)
    {
      real_mode_on = 1U;
      break;
    }
  }

  const float roll_mix = g_roll_pid_output;
  const float pitch_mix = g_pitch_pid_output;
  const float yaw_mix = g_yaw_pid_output;

  float motor_norm[MOTOR_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};

  if (g_motor_gui_test_active != 0U)
  {
    if ((int32_t)(now_ms - g_motor_gui_test_expire_ms) >= 0)
    {
      g_motor_gui_test_active = 0U;
      MotorControl_Disarm();
      return;
    }

    g_motor_output_mode = MOTOR_OUTPUT_MODE_REAL_THROTTLE;
    g_motors_armed = 1U;
    if (g_motor_hw_ready == 0U)
    {
      MotorOutput_InitAndStart();
      if (g_motor_hw_ready == 0U)
      {
        MotorControl_Disarm();
        return;
      }
    }

    for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
    {
      g_motor_cmd_us[i] = MOTOR_PWM_MIN_US;
    }
    g_motor_cmd_us[g_motor_gui_test_index] = g_motor_gui_test_us;
    MotorOutput_WriteMicroseconds(g_motor_cmd_us);
    return;
  }

#if (MOTOR_PWM_SELF_TEST_ENABLE != 0U)
  g_motor_output_mode = MOTOR_OUTPUT_MODE_REAL_THROTTLE;
  g_motors_armed = 1U;

  if (g_motor_hw_ready == 0U)
  {
    MotorOutput_InitAndStart();
    if (g_motor_hw_ready == 0U)
    {
      MotorControl_Disarm();
      return;
    }
  }

  for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
  {
    g_motor_cmd_us[i] = MOTOR_PWM_SELF_TEST_US;
  }
  MotorOutput_WriteMicroseconds(g_motor_cmd_us);
  return;
#endif

  if ((g_rc_link_ok == 0U) || (arm_switch_on == 0U))
  {
    g_motor_output_mode = MOTOR_OUTPUT_MODE_SIMULATION;
    MotorControl_Disarm();
    return;
  }

  if (g_motors_armed == 0U)
  {
    g_motors_armed = 1U;
  }

  if (g_motors_armed == 0U)
  {
    g_motor_output_mode = MOTOR_OUTPUT_MODE_SIMULATION;
    MotorControl_Disarm();
    return;
  }

#if (MOTOR_FORCE_ALL_TEST_ENABLE != 0U)
  g_motor_output_mode = MOTOR_OUTPUT_MODE_REAL_THROTTLE;
  if (g_motor_hw_ready == 0U)
  {
    MotorOutput_InitAndStart();
    if (g_motor_hw_ready == 0U)
    {
      g_motor_output_mode = MOTOR_OUTPUT_MODE_SIMULATION;
      MotorControl_Disarm();
      return;
    }
  }

  for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
  {
    g_motor_cmd_us[i] = MOTOR_FORCE_ALL_TEST_US;
  }
  MotorOutput_WriteMicroseconds(g_motor_cmd_us);
  return;
#endif

  g_motor_output_mode = (real_mode_on != 0U) ? MOTOR_OUTPUT_MODE_REAL_THROTTLE : MOTOR_OUTPUT_MODE_SIMULATION;

  if (g_motor_output_mode == MOTOR_OUTPUT_MODE_REAL_THROTTLE)
  {
    if (g_motor_hw_ready == 0U)
    {
      MotorOutput_InitAndStart();
      if (g_motor_hw_ready == 0U)
      {
        g_motor_output_mode = MOTOR_OUTPUT_MODE_SIMULATION;
        MotorControl_Disarm();
        return;
      }
    }

#if (MOTOR_FIXED_TEST_ENABLE != 0U)
    if (fixed_test_on != 0U)
    {
      const uint16_t select_us = g_rc_channels_us[MOTOR_FIXED_TEST_SELECT_CHANNEL];
      uint8_t selected_motor = 0U;

      if (select_us < 1250U)
      {
        selected_motor = 0U;
      }
      else if (select_us < 1500U)
      {
        selected_motor = 1U;
      }
      else if (select_us < 1750U)
      {
        selected_motor = 2U;
      }
      else
      {
        selected_motor = 3U;
      }

      for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
      {
        g_motor_cmd_us[i] = MOTOR_PWM_MIN_US;
      }
      g_motor_cmd_us[selected_motor] = MOTOR_FIXED_TEST_US;

      MotorOutput_WriteMicroseconds(g_motor_cmd_us);
      return;
    }
#else
    UNUSED(fixed_test_on);
#endif

    float mix_min = 1.0f;
    float mix_max = 0.0f;

    /* Closed-loop Quad-X mix in INAV motor order: M1 AR, M2 FR, M3 AL, M4 FL. */
    motor_norm[0] = throttle - pitch_mix + roll_mix + yaw_mix; /* M1 aft-right */
    motor_norm[1] = throttle + pitch_mix + roll_mix - yaw_mix; /* M2 forward-right */
    motor_norm[2] = throttle - pitch_mix - roll_mix - yaw_mix; /* M3 aft-left */
    motor_norm[3] = throttle + pitch_mix - roll_mix + yaw_mix; /* M4 forward-left */

    for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
    {
      if (motor_norm[i] < mix_min)
      {
        mix_min = motor_norm[i];
      }
      if (motor_norm[i] > mix_max)
      {
        mix_max = motor_norm[i];
      }
    }

    if ((mix_min < 0.0f) || (mix_max > 1.0f))
    {
      const float range = mix_max - mix_min;
      if (range > 1.0f)
      {
        for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
        {
          motor_norm[i] = (motor_norm[i] - mix_min) / range;
        }
      }
      else
      {
        const float shift_up = (mix_min < 0.0f) ? (-mix_min) : 0.0f;
        const float shift_down = (mix_max > 1.0f) ? (mix_max - 1.0f) : 0.0f;
        const float shift = shift_up - shift_down;
        for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
        {
          motor_norm[i] += shift;
        }
      }
    }

    for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
    {
      const float clamped = MotorControl_ClampF(motor_norm[i], 0.0f, 1.0f);
      uint16_t motor_us = (uint16_t)(MOTOR_COMMAND_MIN_US + (uint16_t)lroundf(clamped * (float)(MOTOR_PWM_MAX_US - MOTOR_COMMAND_MIN_US)));
      if ((motor_us > MOTOR_COMMAND_MIN_US) && (motor_us < MOTOR_SPIN_FLOOR_US))
      {
        motor_us = MOTOR_SPIN_FLOOR_US;
      }
      g_motor_cmd_us[i] = motor_us;
    }

    MotorOutput_WriteMicroseconds(g_motor_cmd_us);
    return;
  }

  /* Quad-X software mix in INAV motor order: M1 AR, M2 FR, M3 AL, M4 FL. */
  motor_norm[0] = throttle - pitch_mix + roll_mix + yaw_mix; /* M1 aft-right */
  motor_norm[1] = throttle + pitch_mix + roll_mix - yaw_mix; /* M2 forward-right */
  motor_norm[2] = throttle - pitch_mix - roll_mix - yaw_mix; /* M3 aft-left */
  motor_norm[3] = throttle + pitch_mix - roll_mix + yaw_mix; /* M4 forward-left */

  for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
  {
    const float clamped = MotorControl_ClampF(motor_norm[i], 0.0f, 1.0f);
    g_motor_cmd_us[i] = (uint16_t)(MOTOR_COMMAND_MIN_US + (uint16_t)lroundf(clamped * (float)(MOTOR_PWM_MAX_US - MOTOR_COMMAND_MIN_US)));
  }

  MotorOutput_WriteMicroseconds(g_motor_cmd_us);
}

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

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
#if (USB_ENUM_DIAGNOSTIC == 0U)
  MX_I2C1_Init();
  g_battery_adc_ready = MX_ADC1_Init();
  MX_USART6_UART_Init();
  MX_USART1_UART_Init();
  MX_UART4_Init();
#if (MOTOR_INIT_AT_BOOT != 0U)
  MotorOutput_InitAndStart();
#endif
#endif
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  uint32_t next_imu_probe_ms = 0U;
  uint32_t next_baro_probe_ms = 0U;
  uint32_t next_baro_update_ms = 0U;
  uint32_t next_battery_update_ms = 0U;
  uint32_t now_ms = 0U;
  uint32_t next_telemetry_ms = 0U;
  uint32_t next_tm_link_ms = 0U;

  RateController_Init(&g_rate_controller);

  g_usb_ready = 1U;

#if (USB_ENUM_DIAGNOSTIC == 0U)
  (void)IMU_TryBringup();
  (void)BMP280_Init();
  next_imu_probe_ms = HAL_GetTick() + 500U;
  next_baro_probe_ms = HAL_GetTick() + 1000U;
  next_baro_update_ms = HAL_GetTick();
  next_battery_update_ms = HAL_GetTick();
  next_telemetry_ms = HAL_GetTick();
  next_tm_link_ms = HAL_GetTick();

  (void)TelemetryLink_Init(&huart6);

  MotorOutput_BootHold(1000U);

  /* Keep USB telemetry alive even if CRSF UART RX cannot start. */
  if (HAL_UART_Receive_IT(&huart4, &g_uart4_rx_byte, 1U) != HAL_OK)
  {
    g_rc_link_ok = 0U;
  }
#else
  MotorOutput_BootHold(1000U);
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if (USB_ENUM_DIAGNOSTIC != 0U)
    HAL_Delay(10);
#else
    int16_t gyro_x = 0;
    int16_t gyro_y = 0;
    int16_t gyro_z = 0;
    int16_t accel_x = 0;
    int16_t accel_y = 0;
    int16_t accel_z = 0;
    uint8_t sample_valid = 0U;

    CRSF_ProcessUart();
    MotorGui_ProcessUsbCommands();
    TelemetryLink_Update();
    now_ms = HAL_GetTick();
    if ((g_rc_last_frame_ms != 0U) && ((now_ms - g_rc_last_frame_ms) <= 100U))
    {
      g_rc_link_ok = 1U;
    }
    else
    {
      g_rc_link_ok = 0U;
      RC_SetFailsafeNeutral();
      if (g_motor_gui_test_active == 0U)
      {
        MotorControl_Disarm();
      }
    }

    MotorControl_UpdateVirtual();

    if ((g_imu_found == 0U) && (HAL_GetTick() >= next_imu_probe_ms))
    {
      (void)IMU_TryBringup();
      next_imu_probe_ms = HAL_GetTick() + 500U;
    }

    if ((g_bmp280_found == 0U) && (HAL_GetTick() >= next_baro_probe_ms))
    {
      (void)BMP280_Init();
      next_baro_probe_ms = HAL_GetTick() + 1000U;
      next_baro_update_ms = HAL_GetTick();
    }

    if ((g_bmp280_found != 0U) && (HAL_GetTick() >= next_baro_update_ms))
    {
      if (BMP280_Update() == 0U)
      {
        g_bmp280_found = 0U;
        next_baro_probe_ms = HAL_GetTick() + 1000U;
      }
      else
      {
        next_baro_update_ms = HAL_GetTick() + 25U;
      }
    }

    if ((g_battery_adc_ready != 0U) && (HAL_GetTick() >= next_battery_update_ms))
    {
      Battery_Update();
      next_battery_update_ms = HAL_GetTick() + 100U;
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
      g_display_filter_gx = 0;
      g_display_filter_gy = 0;
      g_display_filter_gz = 0;
      g_gyro_roll_rate_dps = 0.0f;
      g_gyro_pitch_rate_dps = 0.0f;
      g_gyro_yaw_rate_dps = 0.0f;
    }

    {
      static uint32_t s_rate_last_update_ms = 0U;
      uint32_t rate_now_ms = HAL_GetTick();
      float rate_dt = 0.01f;
      uint16_t loop_dt_us = 10000U;

      if (s_rate_last_update_ms != 0U)
      {
        uint32_t delta_ms = rate_now_ms - s_rate_last_update_ms;
        rate_dt = (float)delta_ms * 0.001f;
      }
      s_rate_last_update_ms = rate_now_ms;

      if (rate_dt < 0.0005f)
      {
        rate_dt = 0.0005f;
      }
      else if (rate_dt > 0.02f)
      {
        rate_dt = 0.02f;
      }

      {
        uint32_t dt_us_u32 = (uint32_t)lroundf(rate_dt * 1000000.0f);
        if (dt_us_u32 > 65535U)
        {
          dt_us_u32 = 65535U;
        }
        loop_dt_us = (uint16_t)dt_us_u32;
      }

      if ((sample_valid != 0U) && (g_imu_found != 0U))
      {
        RateController_Update(&g_rate_controller,
                              rate_dt,
                              g_desired_roll_rate_dps,
                              g_desired_pitch_rate_dps,
                              g_desired_yaw_rate_dps,
                              g_gyro_roll_rate_dps,
                              g_gyro_pitch_rate_dps,
                              g_gyro_yaw_rate_dps,
                              &g_roll_pid_output,
                              &g_pitch_pid_output,
                              &g_yaw_pid_output);

        g_roll_rate_error_dps = g_desired_roll_rate_dps - g_gyro_roll_rate_dps;
        g_pitch_rate_error_dps = g_desired_pitch_rate_dps - g_gyro_pitch_rate_dps;
        g_yaw_rate_error_dps = g_desired_yaw_rate_dps - g_gyro_yaw_rate_dps;
      }
      else
      {
        RateController_Reset(&g_rate_controller);
        g_roll_rate_error_dps = 0.0f;
        g_pitch_rate_error_dps = 0.0f;
        g_yaw_rate_error_dps = 0.0f;
        g_roll_pid_output = 0.0f;
        g_pitch_pid_output = 0.0f;
        g_yaw_pid_output = 0.0f;
      }

      if ((g_motors_armed == 0U) || (g_rc_link_ok == 0U) || (g_rc_channels_us[2] <= (MOTOR_THROTTLE_MIN_US + 25U)))
      {
        RateController_Reset(&g_rate_controller);
      }

      now_ms = HAL_GetTick();
      if ((int32_t)(now_ms - next_tm_link_ms) >= 0)
      {
        TelemetryCombinedFast tm_fast;
        memset(&tm_fast, 0, sizeof(tm_fast));

        tm_fast.gyroXdps10 = (int16_t)lroundf(g_gyro_roll_rate_dps * 10.0f);
        tm_fast.gyroYdps10 = (int16_t)lroundf(g_gyro_pitch_rate_dps * 10.0f);
        tm_fast.gyroZdps10 = (int16_t)lroundf(g_gyro_yaw_rate_dps * 10.0f);
        tm_fast.rollCd = (int16_t)g_roll_cd;
        tm_fast.pitchCd = (int16_t)g_pitch_cd;
        tm_fast.yawCd = (int16_t)g_yaw_cd;

        tm_fast.rcThrottleUs = g_rc_channels_us[2];
        tm_fast.rcRollUs = g_rc_channels_us[0];
        tm_fast.rcPitchUs = g_rc_channels_us[1];
        tm_fast.rcYawUs = g_rc_channels_us[3];
        tm_fast.rcArmUs = g_rc_channels_us[4];

        tm_fast.motor1Us = g_motor_cmd_us[0];
        tm_fast.motor2Us = g_motor_cmd_us[1];
        tm_fast.motor3Us = g_motor_cmd_us[2];
        tm_fast.motor4Us = g_motor_cmd_us[3];

        tm_fast.loopDtUs = loop_dt_us;
        tm_fast.armed = g_motors_armed;
        tm_fast.failsafeFlags = (g_rc_link_ok != 0U) ? 0U : 1U;

        (void)TelemetryLink_SendCombinedFast(&tm_fast);
        next_tm_link_ms = now_ms + 20U;
      }
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
                           g_display_filter_gx,
                           g_display_filter_gy,
                           g_display_filter_gz,
                           accel_x,
                           accel_y,
                           accel_z,
                           g_baro_pressure_pa,
                           g_baro_altitude_cm);
      next_telemetry_ms = now_ms + 10U;
    }
#endif
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
  else if (huart->Instance == USART6)
  {
    TelemetryLink_OnUartRxCplt(huart);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6)
  {
    TelemetryLink_OnUartTxCplt(huart);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART4)
  {
    (void)HAL_UART_AbortReceive(huart);
    (void)HAL_UART_Receive_IT(&huart4, &g_uart4_rx_byte, 1U);
  }
  else if (huart->Instance == USART6)
  {
    TelemetryLink_OnUartError(huart);
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
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
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

  /* Keep HSI48 tightly locked for USB FS signaling. */
  __HAL_RCC_CRS_CLK_ENABLE();
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB1;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000U, 1000U);
  RCC_CRSInitStruct.ErrorLimitValue = RCC_CRS_ERRORLIMIT_DEFAULT;
  RCC_CRSInitStruct.HSI48CalibrationValue = RCC_CRS_HSI48CALIBRATION_DEFAULT;
  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10909CECU;
  hi2c1.Init.OwnAddress1 = 0U;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0U;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0U) != HAL_OK)
  {
    Error_Handler();
  }
}

static uint8_t MX_ADC1_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  uint32_t start_ms = HAL_GetTick();

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_ADC12_CLK_ENABLE();

  GPIO_InitStruct.Pin = BATTERY_ADC_PIN | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BATTERY_ADC_PORT, &GPIO_InitStruct);

  /* Exit deep-power-down, enable regulator, and wait briefly. */
  ADC1->CR &= ~ADC_CR_DEEPPWD;
  ADC1->CR |= ADC_CR_ADVREGEN;
  while ((HAL_GetTick() - start_ms) < 10U) {}

  /* Ensure ADC is disabled before calibration/configuration. */
  if ((ADC1->CR & ADC_CR_ADEN) != 0U)
  {
    ADC1->CR |= ADC_CR_ADDIS;
    start_ms = HAL_GetTick();
    while ((ADC1->CR & ADC_CR_ADEN) != 0U)
    {
      if ((HAL_GetTick() - start_ms) > 10U)
      {
        break;
      }
    }
  }

  /* Single-ended calibration. */
  ADC1->CR &= ~ADC_CR_ADCALDIF;
  ADC1->CR |= ADC_CR_ADCAL;
  start_ms = HAL_GetTick();
  while ((ADC1->CR & ADC_CR_ADCAL) != 0U)
  {
    if ((HAL_GetTick() - start_ms) > 100U)
    {
      break;
    }
  }

  /* Select channel, sample time, and one conversion in regular sequence. */
  ADC12_COMMON->CCR = 0U;
  ADC12_COMMON->CCR |= (1UL << ADC_CCR_CKMODE_Pos); /* HCLK/2 kernel clock */

  ADC1->CFGR = 0U;
  ADC1->CFGR |= ADC_CFGR_RES_1; /* 12-bit resolution */
  ADC1->CFGR &= ~ADC_CFGR_EXTEN; /* Software trigger */
  ADC1->CFGR &= ~ADC_CFGR_CONT;  /* Single conversion */
  Battery_SelectAdcChannel(g_battery_adc_channel_index);

  for (uint8_t attempt = 0U; attempt < 3U; attempt++)
  {
    ADC1->ISR = ADC_ISR_ADRDY;
    ADC1->CR |= ADC_CR_ADEN;
    start_ms = HAL_GetTick();
    while ((ADC1->ISR & ADC_ISR_ADRDY) == 0U)
    {
      if ((HAL_GetTick() - start_ms) > 100U)
      {
        break;
      }
    }

    if ((ADC1->ISR & ADC_ISR_ADRDY) != 0U)
    {
      return 1U;
    }

    ADC1->CR |= ADC_CR_ADDIS;
    start_ms = HAL_GetTick();
    while ((ADC1->CR & ADC_CR_ADEN) != 0U)
    {
      if ((HAL_GetTick() - start_ms) > 20U)
      {
        break;
      }
    }
  }

  return 0U;
}

static void Battery_SelectAdcChannel(uint8_t channel_index)
{
  uint32_t smpr_pos = 0U;
  uint32_t smpr_mask = 0U;

  if (channel_index > 18U)
  {
    channel_index = 10U;
  }

  g_battery_adc_channel_index = channel_index;

  ADC1->PCSEL = (1UL << channel_index);

  if (channel_index <= 9U)
  {
    smpr_pos = ((uint32_t)channel_index) * 3U;
    smpr_mask = (0x7UL << smpr_pos);
    ADC1->SMPR1 = (ADC1->SMPR1 & (~smpr_mask)) | (0x7UL << smpr_pos);
  }
  else
  {
    smpr_pos = ((uint32_t)channel_index - 10U) * 3U;
    smpr_mask = (0x7UL << smpr_pos);
    ADC1->SMPR2 = (ADC1->SMPR2 & (~smpr_mask)) | (0x7UL << smpr_pos);
  }

  ADC1->SQR1 = 0U;
  ADC1->SQR1 |= ((uint32_t)channel_index << ADC_SQR1_SQ1_Pos);
}

static void Battery_Update(void)
{
  uint32_t raw = 0U;
  uint32_t max_raw = 0U;
  uint8_t max_channel = g_battery_adc_channel_index;
  uint8_t convert_ok = 0U;
  uint32_t pin_mv = 0U;
  uint32_t battery_mv = 0U;
  static const uint8_t battery_channel_candidates[] = {10U, 11U, 8U, 12U, 13U};
  const uint8_t battery_channel_count = (uint8_t)(sizeof(battery_channel_candidates) / sizeof(battery_channel_candidates[0]));

  if (g_battery_adc_ready == 0U)
  {
    g_battery_adc_raw_last = 0U;
    g_battery_voltage_mv = 0U;
    return;
  }

  for (uint8_t i = 0U; i < battery_channel_count; i++)
  {
    uint8_t candidate_ok = 0U;
    uint8_t candidate_channel = battery_channel_candidates[i];
    uint32_t candidate_raw = Battery_ReadChannelRaw(candidate_channel, &candidate_ok);
    if (candidate_ok != 0U)
    {
      convert_ok = 1U;
      if (candidate_raw >= max_raw)
      {
        max_raw = candidate_raw;
        max_channel = candidate_channel;
      }
    }
  }

  if (convert_ok == 0U)
  {
    g_battery_adc_raw_last = 0U;
    g_battery_voltage_mv = 0U;
    return;
  }

  raw = max_raw;
  g_battery_adc_channel_index = max_channel;
  g_battery_adc_raw_last = (uint16_t)raw;

  if (raw <= 2U)
  {
    if (g_battery_adc_zero_streak < 255U)
    {
      g_battery_adc_zero_streak++;
    }

    if (g_battery_adc_zero_streak >= 20U)
    {
      g_battery_adc_zero_streak = 0U;
      g_battery_adc_candidate_pos = (uint8_t)((g_battery_adc_candidate_pos + 1U) % battery_channel_count);
      Battery_SelectAdcChannel(battery_channel_candidates[g_battery_adc_candidate_pos]);
    }
  }
  else
  {
    g_battery_adc_zero_streak = 0U;
  }

  pin_mv = (raw * BATTERY_VREF_MV) / BATTERY_ADC_MAX_COUNTS;
  battery_mv = (pin_mv * BATTERY_DIVIDER_NUM) / BATTERY_DIVIDER_DEN;
  g_battery_voltage_mv = (uint16_t)battery_mv;
}

static uint32_t Battery_ReadChannelRaw(uint8_t channel_index, uint8_t *ok)
{
  uint32_t raw = 0U;
  uint32_t start_ms = HAL_GetTick();

  if (ok != NULL)
  {
    *ok = 0U;
  }

  Battery_SelectAdcChannel(channel_index);
  ADC1->ISR = ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_EOSMP | ADC_ISR_OVR;
  ADC1->CR |= ADC_CR_ADSTART;

  while ((ADC1->ISR & ADC_ISR_EOC) == 0U)
  {
    if ((HAL_GetTick() - start_ms) > 5U)
    {
      return 0U;
    }
  }

  raw = ADC1->DR;
  if (raw > BATTERY_ADC_MAX_COUNTS)
  {
    raw = BATTERY_ADC_MAX_COUNTS;
  }

  if (ok != NULL)
  {
    *ok = 1U;
  }
  return raw;
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
    huart6.Init.BaudRate = 420000;
    huart6.Init.WordLength = UART_WORDLENGTH_8B;
    huart6.Init.StopBits = UART_STOPBITS_1;
    huart6.Init.Parity = UART_PARITY_NONE;
    huart6.Init.Mode = UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;
    huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart6.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
    if (HAL_UARTEx_SetTxFifoThreshold(&huart6, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
    if (HAL_UARTEx_SetRxFifoThreshold(&huart6, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
    if (HAL_UARTEx_DisableFifoMode(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
    /* USER CODE BEGIN USART6_Init 2 */

    /* USER CODE END USART6_Init 2 */

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
                                 int16_t gx_filtered, int16_t gy_filtered, int16_t gz_filtered,
                                 int16_t ax_raw, int16_t ay_raw, int16_t az_raw,
                                 uint32_t pressure_pa, int32_t altitude_cm)
{
  uint8_t frame[4U + TELEMETRY_PACKET_PAYLOAD_BYTES_V8 + 1U];
  uint8_t checksum = 0U;
  uint8_t payload_index = 0U;

  frame[0] = TELEMETRY_PACKET_SOF1;
  frame[1] = TELEMETRY_PACKET_SOF2;
  frame[2] = TELEMETRY_PACKET_TYPE_TELEMETRY;
  frame[3] = TELEMETRY_PACKET_PAYLOAD_BYTES_V8;

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
  Packet_PutS16LE(&frame[4U + payload_index], gx_filtered);
  payload_index = (uint8_t)(payload_index + 2U);
  Packet_PutS16LE(&frame[4U + payload_index], gy_filtered);
  payload_index = (uint8_t)(payload_index + 2U);
  Packet_PutS16LE(&frame[4U + payload_index], gz_filtered);
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

  Packet_PutU32LE(&frame[4U + payload_index], pressure_pa);
  payload_index = (uint8_t)(payload_index + 4U);

  Packet_PutS32LE(&frame[4U + payload_index], altitude_cm);
  payload_index = (uint8_t)(payload_index + 4U);

  Packet_PutU16LE(&frame[4U + payload_index], g_battery_voltage_mv);
  payload_index = (uint8_t)(payload_index + 2U);

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

  frame[4U + payload_index] = g_motor_output_mode;
  payload_index = (uint8_t)(payload_index + 1U);
  frame[4U + payload_index] = g_motors_armed;
  payload_index = (uint8_t)(payload_index + 1U);
  frame[4U + payload_index] = g_motor_gui_test_active;
  payload_index = (uint8_t)(payload_index + 1U);
  for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
  {
    Packet_PutU16LE(&frame[4U + payload_index], g_motor_cmd_us[i]);
    payload_index = (uint8_t)(payload_index + 2U);
  }

  Packet_PutU16LE(&frame[4U + payload_index], (uint16_t)MotorControl_ClampF(g_display_filter_cutoff_hz, 0.0f, 10.0f));
  payload_index = (uint8_t)(payload_index + 2U);

  Packet_PutU16LE(&frame[4U + payload_index], g_battery_adc_raw_last);
  payload_index = (uint8_t)(payload_index + 2U);
  frame[4U + payload_index] = g_battery_adc_channel_index;
  payload_index = (uint8_t)(payload_index + 1U);
  frame[4U + payload_index] = g_battery_adc_ready;
  payload_index = (uint8_t)(payload_index + 1U);

  Packet_PutS16LE(&frame[4U + payload_index], (int16_t)lroundf(g_desired_roll_rate_dps * 10.0f));
  payload_index = (uint8_t)(payload_index + 2U);
  Packet_PutS16LE(&frame[4U + payload_index], (int16_t)lroundf(g_desired_pitch_rate_dps * 10.0f));
  payload_index = (uint8_t)(payload_index + 2U);
  Packet_PutS16LE(&frame[4U + payload_index], (int16_t)lroundf(g_desired_yaw_rate_dps * 10.0f));
  payload_index = (uint8_t)(payload_index + 2U);

  Packet_PutS16LE(&frame[4U + payload_index], (int16_t)lroundf(g_gyro_roll_rate_dps * 10.0f));
  payload_index = (uint8_t)(payload_index + 2U);
  Packet_PutS16LE(&frame[4U + payload_index], (int16_t)lroundf(g_gyro_pitch_rate_dps * 10.0f));
  payload_index = (uint8_t)(payload_index + 2U);
  Packet_PutS16LE(&frame[4U + payload_index], (int16_t)lroundf(g_gyro_yaw_rate_dps * 10.0f));
  payload_index = (uint8_t)(payload_index + 2U);

  Packet_PutS16LE(&frame[4U + payload_index], (int16_t)lroundf(g_roll_rate_error_dps * 10.0f));
  payload_index = (uint8_t)(payload_index + 2U);
  Packet_PutS16LE(&frame[4U + payload_index], (int16_t)lroundf(g_pitch_rate_error_dps * 10.0f));
  payload_index = (uint8_t)(payload_index + 2U);
  Packet_PutS16LE(&frame[4U + payload_index], (int16_t)lroundf(g_yaw_rate_error_dps * 10.0f));
  payload_index = (uint8_t)(payload_index + 2U);

  Packet_PutS16LE(&frame[4U + payload_index], (int16_t)lroundf(g_roll_pid_output * 10000.0f));
  payload_index = (uint8_t)(payload_index + 2U);
  Packet_PutS16LE(&frame[4U + payload_index], (int16_t)lroundf(g_pitch_pid_output * 10000.0f));
  payload_index = (uint8_t)(payload_index + 2U);
  Packet_PutS16LE(&frame[4U + payload_index], (int16_t)lroundf(g_yaw_pid_output * 10000.0f));
  payload_index = (uint8_t)(payload_index + 2U);

  for (uint32_t i = 2U; i < (4U + TELEMETRY_PACKET_PAYLOAD_BYTES_V8); i++)
  {
    checksum ^= frame[i];
  }

  frame[4U + TELEMETRY_PACKET_PAYLOAD_BYTES_V8] = checksum;

  Serial_Write(frame, (uint16_t)sizeof(frame));
}

static HAL_StatusTypeDef BMP280_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len)
{
  if ((data == NULL) || (len == 0U))
  {
    return HAL_ERROR;
  }

  return HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(g_bmp280_addr << 1), reg,
                          I2C_MEMADD_SIZE_8BIT, data, len, BMP280_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef BMP280_WriteReg(uint8_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(g_bmp280_addr << 1), reg,
                           I2C_MEMADD_SIZE_8BIT, &value, 1U, BMP280_I2C_TIMEOUT_MS);
}

static float BMP280_PressureToAltitudeCm(float pressure_pa, float reference_pressure_pa)
{
  if ((pressure_pa <= 0.0f) || (reference_pressure_pa <= 0.0f))
  {
    return 0.0f;
  }

  return 4433000.0f * (1.0f - powf(pressure_pa / reference_pressure_pa, 0.1903f));
}

static uint8_t BMP280_Init(void)
{
  const uint8_t candidates[2] = {0x76U, 0x77U};
  uint8_t id = 0U;
  uint8_t calib[24] = {0U};

  g_bmp280_found = 0U;

  for (uint8_t i = 0U; i < 2U; i++)
  {
    g_bmp280_addr = candidates[i];
    if (BMP280_ReadRegs(BMP280_REG_ID, &id, 1U) != HAL_OK)
    {
      continue;
    }
    if (id != BMP280_CHIP_ID)
    {
      continue;
    }

    if (BMP280_ReadRegs(BMP280_REG_CALIB_START, calib, (uint16_t)sizeof(calib)) != HAL_OK)
    {
      continue;
    }

    g_bmp280_dig_t1 = (uint16_t)((uint16_t)calib[0] | ((uint16_t)calib[1] << 8U));
    g_bmp280_dig_t2 = (int16_t)((uint16_t)calib[2] | ((uint16_t)calib[3] << 8U));
    g_bmp280_dig_t3 = (int16_t)((uint16_t)calib[4] | ((uint16_t)calib[5] << 8U));
    g_bmp280_dig_p1 = (uint16_t)((uint16_t)calib[6] | ((uint16_t)calib[7] << 8U));
    g_bmp280_dig_p2 = (int16_t)((uint16_t)calib[8] | ((uint16_t)calib[9] << 8U));
    g_bmp280_dig_p3 = (int16_t)((uint16_t)calib[10] | ((uint16_t)calib[11] << 8U));
    g_bmp280_dig_p4 = (int16_t)((uint16_t)calib[12] | ((uint16_t)calib[13] << 8U));
    g_bmp280_dig_p5 = (int16_t)((uint16_t)calib[14] | ((uint16_t)calib[15] << 8U));
    g_bmp280_dig_p6 = (int16_t)((uint16_t)calib[16] | ((uint16_t)calib[17] << 8U));
    g_bmp280_dig_p7 = (int16_t)((uint16_t)calib[18] | ((uint16_t)calib[19] << 8U));
    g_bmp280_dig_p8 = (int16_t)((uint16_t)calib[20] | ((uint16_t)calib[21] << 8U));
    g_bmp280_dig_p9 = (int16_t)((uint16_t)calib[22] | ((uint16_t)calib[23] << 8U));

    (void)BMP280_WriteReg(BMP280_REG_RESET, 0xB6U);
    HAL_Delay(3);
    (void)BMP280_WriteReg(BMP280_REG_CONFIG, 0x00U);
    (void)BMP280_WriteReg(BMP280_REG_CTRL_MEAS, 0x27U); /* temp x1, press x1, normal mode */

    g_bmp280_found = 1U;
    if (BMP280_Update() == 0U)
    {
      g_bmp280_found = 0U;
      continue;
    }

    g_baro_reference_pressure_pa = (float)g_baro_pressure_pa;
    g_baro_altitude_cm = 0;
    g_bmp280_found = 1U;
    return 1U;
  }

  return 0U;
}

static uint8_t BMP280_Update(void)
{
  uint8_t data[6] = {0U};
  int32_t adc_p = 0;
  int32_t adc_t = 0;
  int32_t var1 = 0;
  int32_t var2 = 0;
  int64_t var1_64 = 0;
  int64_t var2_64 = 0;
  int64_t p_64 = 0;

  if (g_bmp280_found == 0U)
  {
    return 0U;
  }

  if (BMP280_ReadRegs(BMP280_REG_PRESS_MSB, data, (uint16_t)sizeof(data)) != HAL_OK)
  {
    return 0U;
  }

  adc_p = (int32_t)(((uint32_t)data[0] << 12U) | ((uint32_t)data[1] << 4U) | ((uint32_t)data[2] >> 4U));
  adc_t = (int32_t)(((uint32_t)data[3] << 12U) | ((uint32_t)data[4] << 4U) | ((uint32_t)data[5] >> 4U));

  var1 = ((((adc_t >> 3) - ((int32_t)g_bmp280_dig_t1 << 1)) * (int32_t)g_bmp280_dig_t2) >> 11);
  var2 = (((((adc_t >> 4) - (int32_t)g_bmp280_dig_t1) * ((adc_t >> 4) - (int32_t)g_bmp280_dig_t1)) >> 12) * (int32_t)g_bmp280_dig_t3) >> 14;
  g_baro_t_fine = var1 + var2;

  var1_64 = (int64_t)g_baro_t_fine - 128000LL;
  var2_64 = var1_64 * var1_64 * (int64_t)g_bmp280_dig_p6;
  var2_64 = var2_64 + ((var1_64 * (int64_t)g_bmp280_dig_p5) << 17);
  var2_64 = var2_64 + (((int64_t)g_bmp280_dig_p4) << 35);
  var1_64 = ((var1_64 * var1_64 * (int64_t)g_bmp280_dig_p3) >> 8) + ((var1_64 * (int64_t)g_bmp280_dig_p2) << 12);
  var1_64 = (((((int64_t)1 << 47) + var1_64) * (int64_t)g_bmp280_dig_p1) >> 33);

  if (var1_64 == 0)
  {
    return 0U;
  }

  p_64 = (1048576LL - (int64_t)adc_p);
  p_64 = (((p_64 << 31) - var2_64) * 3125LL) / var1_64;
  var1_64 = (((int64_t)g_bmp280_dig_p9) * (p_64 >> 13) * (p_64 >> 13)) >> 25;
  var2_64 = (((int64_t)g_bmp280_dig_p8) * p_64) >> 19;
  p_64 = ((p_64 + var1_64 + var2_64) >> 8) + (((int64_t)g_bmp280_dig_p7) << 4);

  g_baro_press_raw = adc_p;
  g_baro_temp_raw = adc_t;
  g_baro_pressure_pa = (uint32_t)((float)p_64 / 256.0f);
  g_baro_altitude_cm = (int32_t)lroundf(BMP280_PressureToAltitudeCm((float)g_baro_pressure_pa, g_baro_reference_pressure_pa));

  return 1U;
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

  DisplayFilter_ProcessGyro(dt, gx, gy, gz, &g_display_filter_gx, &g_display_filter_gy, &g_display_filter_gz);

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
  g_gyro_roll_rate_dps = gx_aligned;
  g_gyro_pitch_rate_dps = gy_aligned;
  g_gyro_yaw_rate_dps = gz_aligned;
  
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
