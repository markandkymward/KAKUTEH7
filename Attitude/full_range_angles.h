#ifndef FULL_RANGE_ANGLES_H
#define FULL_RANGE_ANGLES_H

#include "FusionMath.h"

typedef struct {
    float roll_full_deg;
    float pitch_full_deg;
    float yaw_relative_deg;
} FullRangeAngles;

FullRangeAngles FullRangeAngles_FromQuaternion(const FusionQuaternion q);

#endif
