#include "dshot.h"
#include "main.h"

#define DSHOT_MOTOR_COUNT          1U
#define DSHOT_BITRATE_HZ           150000U
#define DSHOT_TIMER_CHANNEL        TIM_CHANNEL_3
#define DSHOT_TIMER_GPIO_PORT      GPIOB
#define DSHOT_TIMER_GPIO_PIN       GPIO_PIN_0
#define DSHOT_TIMER_GPIO_AF        GPIO_AF2_TIM3
#define DSHOT_FRAME_BITS           16U
#define DSHOT_RESET_SLOTS          8U
#define DSHOT_FRAME_SLOTS          (DSHOT_FRAME_BITS + DSHOT_RESET_SLOTS)

extern TIM_HandleTypeDef htim3;

static DMA_HandleTypeDef g_hdma_tim3_ch3;
static uint8_t g_dshot_ready = 0U;
static uint8_t g_dshot_dma_started = 0U;
static uint16_t g_dshot_last_values[DSHOT_MOTOR_COUNT] = {0U};
/* DMA on STM32H7 cannot access DTCM; place waveform buffer in RAM_D2. */
__attribute__((section(".ram_d2"), aligned(32)))
static uint32_t g_dshot_buffer_m1[DSHOT_FRAME_SLOTS] = {0U};
static uint32_t g_dshot_period_ticks = 0U;
static uint32_t g_dshot_pulse_0 = 0U;
static uint32_t g_dshot_pulse_1 = 0U;

static uint16_t dshot_encode_value(uint16_t value)
{
  uint16_t packet = (uint16_t)((value & 0x07FFU) << 1U);
  uint16_t csum_data = packet;
  uint8_t csum = 0U;

  for (uint8_t i = 0U; i < 3U; i++)
  {
    csum ^= (uint8_t)(csum_data & 0x000FU);
    csum_data >>= 4U;
  }

  return (uint16_t)((packet << 4U) | (csum & 0x0FU));
}

static void dshot_build_frame(uint16_t value)
{
  uint16_t packet = dshot_encode_value(value);

  for (uint8_t bit = 0U; bit < DSHOT_FRAME_BITS; bit++)
  {
    uint16_t bit_mask = (uint16_t)(1U << (15U - bit));
    g_dshot_buffer_m1[bit] = ((packet & bit_mask) != 0U) ? g_dshot_pulse_1 : g_dshot_pulse_0;
  }

  for (uint8_t i = DSHOT_FRAME_BITS; i < DSHOT_FRAME_SLOTS; i++)
  {
    g_dshot_buffer_m1[i] = 0U;
  }
}

uint8_t dshot_init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  uint32_t tim_clk_hz = HAL_RCC_GetPCLK1Freq();

  g_dshot_ready = 0U;
  g_dshot_dma_started = 0U;
  g_dshot_last_values[0] = 0U;

  if (tim_clk_hz == 0U)
  {
    return 0U;
  }

  g_dshot_period_ticks = (tim_clk_hz + (DSHOT_BITRATE_HZ / 2U)) / DSHOT_BITRATE_HZ;
  if (g_dshot_period_ticks < 16U)
  {
    return 0U;
  }

  g_dshot_pulse_0 = (g_dshot_period_ticks * 3U) / 8U;
  g_dshot_pulse_1 = (g_dshot_period_ticks * 3U) / 4U;
  if (g_dshot_pulse_0 == 0U)
  {
    g_dshot_pulse_0 = 1U;
  }
  if (g_dshot_pulse_1 <= g_dshot_pulse_0)
  {
    g_dshot_pulse_1 = g_dshot_pulse_0 + 1U;
  }

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  GPIO_InitStruct.Pin = DSHOT_TIMER_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = DSHOT_TIMER_GPIO_AF;
  HAL_GPIO_Init(DSHOT_TIMER_GPIO_PORT, &GPIO_InitStruct);

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0U;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = g_dshot_period_ticks - 1U;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    return 0U;
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0U;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, DSHOT_TIMER_CHANNEL) != HAL_OK)
  {
    return 0U;
  }

  g_hdma_tim3_ch3.Instance = DMA1_Stream0;
  g_hdma_tim3_ch3.Init.Request = DMA_REQUEST_TIM3_CH3;
  g_hdma_tim3_ch3.Init.Direction = DMA_MEMORY_TO_PERIPH;
  g_hdma_tim3_ch3.Init.PeriphInc = DMA_PINC_DISABLE;
  g_hdma_tim3_ch3.Init.MemInc = DMA_MINC_ENABLE;
  g_hdma_tim3_ch3.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  g_hdma_tim3_ch3.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  g_hdma_tim3_ch3.Init.Mode = DMA_CIRCULAR;
  g_hdma_tim3_ch3.Init.Priority = DMA_PRIORITY_VERY_HIGH;
  g_hdma_tim3_ch3.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

  (void)HAL_DMA_DeInit(&g_hdma_tim3_ch3);
  if (HAL_DMA_Init(&g_hdma_tim3_ch3) != HAL_OK)
  {
    return 0U;
  }

  __HAL_LINKDMA(&htim3, hdma[TIM_DMA_ID_CC3], g_hdma_tim3_ch3);

  dshot_build_frame(0U);

  if (HAL_TIM_PWM_Start_DMA(&htim3, DSHOT_TIMER_CHANNEL, g_dshot_buffer_m1, DSHOT_FRAME_SLOTS) != HAL_OK)
  {
    return 0U;
  }

  g_dshot_dma_started = 1U;
  g_dshot_ready = 1U;
  return 1U;
}

void dshot_write_motor(uint8_t motor_index, uint16_t value)
{
  uint32_t primask = 0U;

  if ((g_dshot_ready == 0U) || (motor_index >= DSHOT_MOTOR_COUNT))
  {
    return;
  }

  if (value > 2047U)
  {
    value = 2047U;
  }

  g_dshot_last_values[motor_index] = value;

  primask = __get_PRIMASK();
  __disable_irq();
  dshot_build_frame(value);
  if (primask == 0U)
  {
    __enable_irq();
  }
}

void dshot_send_idle(void)
{
  dshot_write_motor(0U, 0U);
}

void dshot_stop_all(void)
{
  if (g_dshot_dma_started != 0U)
  {
    (void)HAL_TIM_PWM_Stop_DMA(&htim3, DSHOT_TIMER_CHANNEL);
    g_dshot_dma_started = 0U;
  }

  (void)HAL_TIM_PWM_Stop(&htim3, DSHOT_TIMER_CHANNEL);
  HAL_GPIO_WritePin(DSHOT_TIMER_GPIO_PORT, DSHOT_TIMER_GPIO_PIN, GPIO_PIN_RESET);
  g_dshot_last_values[0] = 0U;
}

uint8_t dshot_is_ready(void)
{
  return g_dshot_ready;
}

uint8_t dshot_dma_running(void)
{
  return g_dshot_dma_started;
}

uint16_t dshot_last_value(uint8_t motor_index)
{
  if (motor_index >= DSHOT_MOTOR_COUNT)
  {
    return 0U;
  }

  return g_dshot_last_values[motor_index];
}
