#ifndef ATTITUDE_ESTIMATOR_H
#define ATTITUDE_ESTIMATOR_H

#include <stdint.h>
#include "FusionAhrs.h"
#include "full_range_angles.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    FusionQuaternion quaternion;
    FusionVector gravityBody;
    FusionVector bodyDown;
    float rollDeg;
    float pitchDeg;
    float yawRelativeDeg;
    char poseLabel[48];
} AttitudeEstimate;

typedef struct {
    FusionAhrs fusion;
    uint8_t ready;
    AttitudeEstimate estimate;
} AttitudeEstimator;

void AttitudeEstimator_Init(AttitudeEstimator *estimator);
void AttitudeEstimator_Reset(AttitudeEstimator *estimator);
void AttitudeEstimator_Update(AttitudeEstimator *estimator,
                             float dt_s,
                             FusionVector gyro_body_dps,
                             FusionVector accel_body_g);
const AttitudeEstimate *AttitudeEstimator_GetEstimate(const AttitudeEstimator *estimator);

#ifdef __cplusplus
}
#endif

#endif