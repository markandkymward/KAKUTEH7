#ifndef ATTITUDE_ESTIMATOR_H
#define ATTITUDE_ESTIMATOR_H

#include <stdbool.h>
#include <stdint.h>

#include "FusionAhrs.h"
#include "FusionBias.h"
#include "full_range_angles.h"

typedef struct {
    FusionQuaternion quaternion;
    FullRangeAngles full_deg;
    float sample_hz;
    bool valid;
} AttitudeState;

typedef struct {
    FusionAhrs ahrs;
    FusionBias bias;
    AttitudeState state;
} AttitudeEstimator;

void AttitudeEstimator_Init(AttitudeEstimator *estimator, float gain, float accel_rejection, float recovery_trigger_s);
bool AttitudeEstimator_Update(AttitudeEstimator *estimator, FusionVector gyro_deg_s, FusionVector accel_g, float dt_s);
const AttitudeState *AttitudeEstimator_GetState(const AttitudeEstimator *estimator);

#endif
