#ifndef DISPLAY_FILTER_H
#define DISPLAY_FILTER_H

#include <stdint.h>

void DisplayFilter_Init(float cutoff_hz);
void DisplayFilter_SetCutoffHz(float cutoff_hz);
void DisplayFilter_ProcessGyro(float dt_seconds,
                               int16_t raw_gx,
                               int16_t raw_gy,
                               int16_t raw_gz,
                               int16_t *filtered_gx,
                               int16_t *filtered_gy,
                               int16_t *filtered_gz);

#endif /* DISPLAY_FILTER_H */
