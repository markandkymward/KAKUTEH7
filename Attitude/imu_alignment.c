#include "imu_alignment.h"

#include <stdio.h>
#include <string.h>

static ImuAlignment s_active_alignment;
static ImuAxisMap s_map;
static char s_mapping_str[96];

static const ImuAxisMap k_maps[] = {
    [IMU_ALIGN_IDENTITY] = {.x_src = 0, .y_src = 1, .z_src = 2, .x_sign = +1, .y_sign = +1, .z_sign = +1},
    [IMU_ALIGN_CW90] = {.x_src = 1, .y_src = 0, .z_src = 2, .x_sign = +1, .y_sign = -1, .z_sign = +1},
    [IMU_ALIGN_CW180] = {.x_src = 0, .y_src = 1, .z_src = 2, .x_sign = -1, .y_sign = -1, .z_sign = +1},
    [IMU_ALIGN_CW270] = {.x_src = 1, .y_src = 0, .z_src = 2, .x_sign = -1, .y_sign = +1, .z_sign = +1},
    [IMU_ALIGN_CW90_FLIPPED] = {.x_src = 1, .y_src = 0, .z_src = 2, .x_sign = +1, .y_sign = -1, .z_sign = -1},
    [IMU_ALIGN_CW180_FLIPPED] = {.x_src = 0, .y_src = 1, .z_src = 2, .x_sign = -1, .y_sign = -1, .z_sign = -1},
    [IMU_ALIGN_CW270_FLIPPED] = {.x_src = 1, .y_src = 0, .z_src = 2, .x_sign = -1, .y_sign = +1, .z_sign = -1},
    [IMU_ALIGN_CUSTOM] = {
        .x_src = IMU_CUSTOM_MAP_X_SRC,
        .y_src = IMU_CUSTOM_MAP_Y_SRC,
        .z_src = IMU_CUSTOM_MAP_Z_SRC,
        .x_sign = IMU_CUSTOM_MAP_X_SIGN,
        .y_sign = IMU_CUSTOM_MAP_Y_SIGN,
        .z_sign = IMU_CUSTOM_MAP_Z_SIGN,
    },
};

static const char *k_names[] = {
    [IMU_ALIGN_IDENTITY] = "identity",
    [IMU_ALIGN_CW90] = "cw90",
    [IMU_ALIGN_CW180] = "cw180",
    [IMU_ALIGN_CW270] = "cw270",
    [IMU_ALIGN_CW90_FLIPPED] = "flip-cw90",
    [IMU_ALIGN_CW180_FLIPPED] = "flip-cw180",
    [IMU_ALIGN_CW270_FLIPPED] = "flip-cw270",
    [IMU_ALIGN_CUSTOM] = "custom",
};

static float select_axis(const FusionVector v, const int8_t src) {
    if (src == 0) {
        return v.axis.x;
    }
    if (src == 1) {
        return v.axis.y;
    }
    return v.axis.z;
}

static char axis_letter(const int8_t src) {
    return src == 0 ? 'X' : (src == 1 ? 'Y' : 'Z');
}

static void build_mapping_string(void) {
    (void)snprintf(
        s_mapping_str,
        sizeof(s_mapping_str),
        "Body X = %cSensor %c, Body Y = %cSensor %c, Body Z = %cSensor %c",
        s_map.x_sign > 0 ? '+' : '-', axis_letter(s_map.x_src),
        s_map.y_sign > 0 ? '+' : '-', axis_letter(s_map.y_src),
        s_map.z_sign > 0 ? '+' : '-', axis_letter(s_map.z_src));
}

void ImuAlignment_Init(ImuAlignment alignment) {
    ImuAlignment_SetActive(alignment);
}

ImuAlignment ImuAlignment_GetActive(void) {
    return s_active_alignment;
}

void ImuAlignment_SetActive(ImuAlignment alignment) {
    if (alignment < IMU_ALIGN_IDENTITY || alignment > IMU_ALIGN_CUSTOM) {
        alignment = IMU_ALIGN_IDENTITY;
    }
    s_active_alignment = alignment;
    s_map = k_maps[alignment];
    build_mapping_string();
}

const char *ImuAlignment_GetName(ImuAlignment alignment) {
    if (alignment < IMU_ALIGN_IDENTITY || alignment > IMU_ALIGN_CUSTOM) {
        return "unknown";
    }
    return k_names[alignment];
}

const ImuAxisMap *ImuAlignment_GetMap(void) {
    return &s_map;
}

void ImuMapSensorToBody(const FusionVector sensor, FusionVector *body) {
    body->axis.x = (float)s_map.x_sign * select_axis(sensor, s_map.x_src);
    body->axis.y = (float)s_map.y_sign * select_axis(sensor, s_map.y_src);
    body->axis.z = (float)s_map.z_sign * select_axis(sensor, s_map.z_src);
}

void ImuMapGyroSensorToBody(const FusionVector sensor, FusionVector *body) {
    ImuMapSensorToBody(sensor, body);
}

void ImuMapAccelSensorToBody(const FusionVector sensor, FusionVector *body) {
    ImuMapSensorToBody(sensor, body);
}

const char *ImuAlignment_GetSignedAxisMappingString(void) {
    return s_mapping_str;
}

bool ImuAlignment_ParseCommand(const char *token, ImuAlignment *alignment) {
    for (int i = IMU_ALIGN_IDENTITY; i <= IMU_ALIGN_CUSTOM; i++) {
        if (strcmp(token, k_names[i]) == 0) {
            *alignment = (ImuAlignment)i;
            return true;
        }
    }
    return false;
}
