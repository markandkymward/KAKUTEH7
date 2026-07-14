#ifndef IMU_ALIGNMENT_H
#define IMU_ALIGNMENT_H

#include <stdbool.h>
#include <stdint.h>

#include "FusionMath.h"

typedef enum {
    IMU_ALIGN_IDENTITY = 0,
    IMU_ALIGN_CW90,
    IMU_ALIGN_CW180,
    IMU_ALIGN_CW270,
    IMU_ALIGN_CW90_FLIPPED,
    IMU_ALIGN_CW180_FLIPPED,
    IMU_ALIGN_CW270_FLIPPED,
    IMU_ALIGN_CUSTOM
} ImuAlignment;

typedef struct {
    int8_t x_src;
    int8_t y_src;
    int8_t z_src;
    int8_t x_sign;
    int8_t y_sign;
    int8_t z_sign;
} ImuAxisMap;

#ifndef IMU_ALIGNMENT
/*
 * Definitive sensor-to-body mapping from six static accel poses:
 *   Body X = -Sensor Y
 *   Body Y = -Sensor X
 *   Body Z = -Sensor Z
 * This is expressed as a custom map.
 */
#define IMU_ALIGNMENT IMU_ALIGN_CUSTOM
#endif

#ifndef IMU_CUSTOM_MAP_X_SRC
#define IMU_CUSTOM_MAP_X_SRC 1
#define IMU_CUSTOM_MAP_X_SIGN -1
#define IMU_CUSTOM_MAP_Y_SRC 0
#define IMU_CUSTOM_MAP_Y_SIGN -1
#define IMU_CUSTOM_MAP_Z_SRC 2
#define IMU_CUSTOM_MAP_Z_SIGN -1
#endif

void ImuAlignment_Init(ImuAlignment alignment);
ImuAlignment ImuAlignment_GetActive(void);
void ImuAlignment_SetActive(ImuAlignment alignment);
const char *ImuAlignment_GetName(ImuAlignment alignment);
const ImuAxisMap *ImuAlignment_GetMap(void);

void ImuMapSensorToBody(const FusionVector sensor, FusionVector *body);
void ImuMapGyroSensorToBody(const FusionVector sensor, FusionVector *body);
void ImuMapAccelSensorToBody(const FusionVector sensor, FusionVector *body);

const char *ImuAlignment_GetSignedAxisMappingString(void);
bool ImuAlignment_ParseCommand(const char *token, ImuAlignment *alignment);

#endif
