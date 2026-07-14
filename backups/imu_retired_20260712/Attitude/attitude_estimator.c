#include "attitude_estimator.h"

#include <math.h>
#include <string.h>

static void AttitudeEstimator_UpdateDerivedState(AttitudeEstimator *estimator)
{
    FullRangePose pose;

    if (estimator == NULL)
    {
        return;
    }

    estimator->estimate.quaternion = FusionAhrsGetQuaternion(&estimator->fusion);
    estimator->estimate.gravityBody = FusionAhrsGetGravity(&estimator->fusion);

    FullRangeAnglesFromQuaternion(estimator->estimate.quaternion, &pose);

    estimator->estimate.bodyDown = pose.bodyDown;
    estimator->estimate.rollDeg = pose.rollDeg;
    estimator->estimate.pitchDeg = pose.pitchDeg;
    estimator->estimate.yawRelativeDeg = pose.yawRelativeDeg;
    FullRangeAnglesDescribeDiscretePose(estimator->estimate.bodyDown,
                                        estimator->estimate.poseLabel,
                                        sizeof(estimator->estimate.poseLabel));
}

void AttitudeEstimator_Init(AttitudeEstimator *estimator)
{
    FusionAhrsSettings settings;

    if (estimator == NULL)
    {
        return;
    }

    memset(estimator, 0, sizeof(*estimator));

    FusionAhrsInitialise(&estimator->fusion);

    settings = fusionAhrsDefaultSettings;
    settings.convention = FusionConventionNwu;
    settings.gain = 0.45f;
    settings.gyroscopeRange = 2000.0f;
    settings.accelerationRejection = 90.0f;
    settings.magneticRejection = 0.0f;
    settings.recoveryTriggerPeriod = 100U;
    FusionAhrsSetSettings(&estimator->fusion, &settings);

    estimator->ready = 1U;
    AttitudeEstimator_UpdateDerivedState(estimator);
}

void AttitudeEstimator_Reset(AttitudeEstimator *estimator)
{
    if (estimator == NULL)
    {
        return;
    }

    AttitudeEstimator_Init(estimator);
}

void AttitudeEstimator_Update(AttitudeEstimator *estimator,
                             float dt_s,
                             FusionVector gyro_body_dps,
                             FusionVector accel_body_g)
{
    FusionQuaternion quaternion;

    if (estimator == NULL)
    {
        return;
    }

    if (estimator->ready == 0U)
    {
        AttitudeEstimator_Init(estimator);
    }

    FusionAhrsUpdateNoMagnetometer(&estimator->fusion, gyro_body_dps, accel_body_g, dt_s);
    quaternion = FusionAhrsGetQuaternion(&estimator->fusion);

    if ((!isfinite(quaternion.element.w)) ||
        (!isfinite(quaternion.element.x)) ||
        (!isfinite(quaternion.element.y)) ||
        (!isfinite(quaternion.element.z)))
    {
        AttitudeEstimator_Init(estimator);
        return;
    }

    AttitudeEstimator_UpdateDerivedState(estimator);
}

const AttitudeEstimate *AttitudeEstimator_GetEstimate(const AttitudeEstimator *estimator)
{
    if (estimator == NULL)
    {
        return NULL;
    }

    return &estimator->estimate;
}