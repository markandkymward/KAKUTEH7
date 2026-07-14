#include "full_range_angles.h"

#include <math.h>

/*
 * Full-range definition used here:
 * 1) Compute body-frame direction of NED down vector d_b from quaternion.
 * 2) roll_full  = atan2(d_b.y, d_b.z) in degrees, range (-180, 180].
 * 3) pitch_full = atan2(-d_b.x, d_b.z) in degrees, range (-180, 180].
 *
 * This representation remains informative through inverted attitudes.
 * It is not a standard Tait-Bryan sequence and has expected coupling near
 * near-vertical down-vector projections.
 */

static FusionVector rotate_ned_down_to_body(const FusionQuaternion q) {
    const float qw = q.element.w;
    const float qx = q.element.x;
    const float qy = q.element.y;
    const float qz = q.element.z;

    /* Third row of rotation matrix Earth->Body for NED down axis [0,0,1]. */
    FusionVector down_body = {.axis = {
        .x = 2.0f * (qx * qz - qw * qy),
        .y = 2.0f * (qy * qz + qw * qx),
        .z = 1.0f - 2.0f * (qx * qx + qy * qy),
    }};
    return down_body;
}

static float wrap_deg_180(float angle_deg) {
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static float suppress_cross_axis_ambiguity(const float numer,
                                           const float orth_axis,
                                           const float angle_deg) {
    const float numer_small_threshold = 0.08f;
    const float orth_dominant_threshold = 0.60f;

    /*
     * Ambiguous case: one axis is near zero while the orthogonal tilt axis is
     * dominant (e.g. pitch should stay near 0 during a pure roll through 90).
     * In this region, atan2(numer, denom) can jump between 0 and 180 from noise.
     */
    if ((fabsf(numer) < numer_small_threshold) && (fabsf(orth_axis) > orth_dominant_threshold)) {
        return 0.0f;
    }

    return angle_deg;
}

FullRangeAngles FullRangeAngles_FromQuaternion(const FusionQuaternion q) {
    const FusionVector d = rotate_ned_down_to_body(q);
    const FusionEuler euler = FusionQuaternionToEuler(q);

    FullRangeAngles out;
    out.roll_full_deg = suppress_cross_axis_ambiguity(
        d.axis.y,
        d.axis.x,
        atan2f(d.axis.y, d.axis.z) * (180.0f / (float)M_PI));
    out.pitch_full_deg = suppress_cross_axis_ambiguity(
        d.axis.x,
        d.axis.y,
        atan2f(-d.axis.x, d.axis.z) * (180.0f / (float)M_PI));
    out.yaw_relative_deg = wrap_deg_180(euler.angle.yaw);
    return out;
}
