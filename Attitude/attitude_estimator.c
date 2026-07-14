#include "attitude_estimator.h"

#include <math.h>

static bool quat_is_valid(const FusionQuaternion q) {
    const float n2 = q.element.w * q.element.w + q.element.x * q.element.x +
                     q.element.y * q.element.y + q.element.z * q.element.z;
    return isfinite(n2) && n2 > 0.5f && n2 < 1.5f;
}

void AttitudeEstimator_Init(AttitudeEstimator *estimator, float gain, float accel_rejection, float recovery_trigger_s) {
    FusionAhrsInitialise(&estimator->ahrs);
    FusionBiasInitialise(&estimator->bias);

    const float default_sample_rate = 1000.0f;
    unsigned int recovery_samples = (unsigned int)(recovery_trigger_s * default_sample_rate);
    if (recovery_samples < 1U) {
        recovery_samples = 1U;
    }

    const FusionAhrsSettings settings = {
        .sampleRate = default_sample_rate,
        .convention = FusionConventionNed,
        .gain = gain,
        .gyroscopeRange = 2000.0f,
        .accelerationRejection = accel_rejection,
        .magneticRejection = 0.0f,
        .recoveryTriggerPeriod = recovery_samples,
    };
    FusionAhrsSetSettings(&estimator->ahrs, &settings);

    estimator->state.quaternion = FusionAhrsGetQuaternion(&estimator->ahrs);
    estimator->state.full_deg = FullRangeAngles_FromQuaternion(estimator->state.quaternion);
    estimator->state.sample_hz = 0.0f;
    estimator->state.valid = false;
}

bool AttitudeEstimator_Update(AttitudeEstimator *estimator, FusionVector gyro_deg_s, FusionVector accel_g, float dt_s) {
    if (!(dt_s > 0.0f) || !isfinite(dt_s)) {
        return false;
    }

    const FusionVector gyro_corrected = FusionBiasUpdate(&estimator->bias, gyro_deg_s);

    FusionAhrsSetSamplePeriod(&estimator->ahrs, dt_s);

    FusionAhrsUpdateNoMagnetometer(
        &estimator->ahrs,
        gyro_corrected,
        accel_g);

    const FusionQuaternion q = FusionAhrsGetQuaternion(&estimator->ahrs);
    if (!quat_is_valid(q)) {
        estimator->state.valid = false;
        return false;
    }

    estimator->state.quaternion = q;
    estimator->state.full_deg = FullRangeAngles_FromQuaternion(q);
    estimator->state.sample_hz = 1.0f / dt_s;
    estimator->state.valid = true;
    return true;
}

const AttitudeState *AttitudeEstimator_GetState(const AttitudeEstimator *estimator) {
    return &estimator->state;
}
