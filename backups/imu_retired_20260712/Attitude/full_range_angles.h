#ifndef FULL_RANGE_ANGLES_H
#define FULL_RANGE_ANGLES_H

#include <stddef.h>
#include "FusionMath.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    FusionVector bodyDown;
    float rollDeg;
    float pitchDeg;
    float yawRelativeDeg;
} FullRangePose;

void FullRangeAnglesFromQuaternion(FusionQuaternion quaternion, FullRangePose *pose);
void FullRangeAnglesDescribeDiscretePose(FusionVector body_down, char *label, size_t label_size);

#ifdef __cplusplus
}
#endif

#endif