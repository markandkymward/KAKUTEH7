#include "display_filter.h"

#include <stddef.h>

#define DISPLAY_FILTER_MIN_CUTOFF_HZ  5.0f
#define DISPLAY_FILTER_MAX_CUTOFF_HZ  150.0f
#define DISPLAY_FILTER_TWO_PI         6.2831853071795864769f
#define DISPLAY_FILTER_EPSILON        0.000001f

static float s_cutoff_hz = 40.0f;
static float s_rc_seconds = 1.0f / (DISPLAY_FILTER_TWO_PI * 40.0f);
static float s_state_gx = 0.0f;
static float s_state_gy = 0.0f;
static float s_state_gz = 0.0f;
static uint8_t s_initialized = 0U;

static float display_filter_clamp(float value, float min_value, float max_value)
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

static int16_t display_filter_round_to_i16(float value)
{
  if (value >= 0.0f)
  {
    value += 0.5f;
  }
  else
  {
    value -= 0.5f;
  }

  if (value > 32767.0f)
  {
    value = 32767.0f;
  }
  else if (value < -32768.0f)
  {
    value = -32768.0f;
  }

  return (int16_t)value;
}

static void display_filter_recompute_coefficients(float cutoff_hz)
{
  s_cutoff_hz = display_filter_clamp(cutoff_hz, DISPLAY_FILTER_MIN_CUTOFF_HZ, DISPLAY_FILTER_MAX_CUTOFF_HZ);
  s_rc_seconds = 1.0f / (DISPLAY_FILTER_TWO_PI * s_cutoff_hz);
}

void DisplayFilter_Init(float cutoff_hz)
{
  display_filter_recompute_coefficients(cutoff_hz);
  s_state_gx = 0.0f;
  s_state_gy = 0.0f;
  s_state_gz = 0.0f;
  s_initialized = 0U;
}

void DisplayFilter_SetCutoffHz(float cutoff_hz)
{
  float clamped = display_filter_clamp(cutoff_hz, DISPLAY_FILTER_MIN_CUTOFF_HZ, DISPLAY_FILTER_MAX_CUTOFF_HZ);

  if ((s_cutoff_hz < (clamped - DISPLAY_FILTER_EPSILON)) || (s_cutoff_hz > (clamped + DISPLAY_FILTER_EPSILON)))
  {
    display_filter_recompute_coefficients(clamped);
  }
}

void DisplayFilter_ProcessGyro(float dt_seconds,
                               int16_t raw_gx,
                               int16_t raw_gy,
                               int16_t raw_gz,
                               int16_t *filtered_gx,
                               int16_t *filtered_gy,
                               int16_t *filtered_gz)
{
  float alpha = 0.0f;
  float raw_x = (float)raw_gx;
  float raw_y = (float)raw_gy;
  float raw_z = (float)raw_gz;

  if ((filtered_gx == NULL) || (filtered_gy == NULL) || (filtered_gz == NULL))
  {
    return;
  }

  if (dt_seconds < DISPLAY_FILTER_EPSILON)
  {
    dt_seconds = DISPLAY_FILTER_EPSILON;
  }

  alpha = dt_seconds / (s_rc_seconds + dt_seconds);
  if (alpha < 0.0f)
  {
    alpha = 0.0f;
  }
  else if (alpha > 1.0f)
  {
    alpha = 1.0f;
  }

  if (s_initialized == 0U)
  {
    s_state_gx = raw_x;
    s_state_gy = raw_y;
    s_state_gz = raw_z;
    s_initialized = 1U;
  }
  else
  {
    s_state_gx = s_state_gx + alpha * (raw_x - s_state_gx);
    s_state_gy = s_state_gy + alpha * (raw_y - s_state_gy);
    s_state_gz = s_state_gz + alpha * (raw_z - s_state_gz);
  }

  *filtered_gx = display_filter_round_to_i16(s_state_gx);
  *filtered_gy = display_filter_round_to_i16(s_state_gy);
  *filtered_gz = display_filter_round_to_i16(s_state_gz);
}
