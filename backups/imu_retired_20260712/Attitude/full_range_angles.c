#include "full_range_angles.h"

#include <math.h>
#include <stdio.h>

typedef struct {
    float magnitude;
    int axis;
    float value;
} PoseAxisComponent;

static float FullRangeAngles_WrapDeg(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static const char *FullRangeAngles_AxisLabel(int axis, float value)
{
    switch (axis)
    {
        case 0:
            return (value >= 0.0f) ? "nose-down" : "nose-up";
        case 1:
            return (value >= 0.0f) ? "right-side-down" : "left-side-down";
        case 2:
        default:
            return (value >= 0.0f) ? "upside-down" : "level";
    }
}

static void FullRangeAngles_SortDescending(PoseAxisComponent *a, PoseAxisComponent *b, PoseAxisComponent *c)
{
    PoseAxisComponent tmp;

    if (a->magnitude < b->magnitude)
    {
        tmp = *a;
        *a = *b;
        *b = tmp;
    }
    if (b->magnitude < c->magnitude)
    {
        tmp = *b;
        *b = *c;
        *c = tmp;
    }
    if (a->magnitude < b->magnitude)
    {
        tmp = *a;
        *a = *b;
        *b = tmp;
    }
}

void FullRangeAnglesFromQuaternion(FusionQuaternion quaternion, FullRangePose *pose)
{
    FusionEuler euler;
    FusionVector gravity_body;
    FusionVector body_down;

    if (pose == NULL)
    {
        return;
    }

    gravity_body.axis.x = 2.0f * ((quaternion.element.x * quaternion.element.z) - (quaternion.element.w * quaternion.element.y));
    gravity_body.axis.y = 2.0f * ((quaternion.element.y * quaternion.element.z) + (quaternion.element.w * quaternion.element.x));
    gravity_body.axis.z = 2.0f * ((quaternion.element.w * quaternion.element.w) - 0.5f + (quaternion.element.z * quaternion.element.z));

    body_down.axis.x = -gravity_body.axis.x;
    body_down.axis.y = -gravity_body.axis.y;
    body_down.axis.z = -gravity_body.axis.z;

    euler = FusionQuaternionToEuler(quaternion);

    pose->bodyDown = body_down;
    pose->rollDeg = FullRangeAngles_WrapDeg(FusionRadiansToDegrees(atan2f(body_down.axis.y, -body_down.axis.z)));
    pose->pitchDeg = FullRangeAngles_WrapDeg(FusionRadiansToDegrees(atan2f(-body_down.axis.x, -body_down.axis.z)));
    pose->yawRelativeDeg = FullRangeAngles_WrapDeg(euler.angle.yaw);
}

void FullRangeAnglesDescribeDiscretePose(FusionVector body_down, char *label, size_t label_size)
{
    PoseAxisComponent x_component = { fabsf(body_down.axis.x), 0, body_down.axis.x };
    PoseAxisComponent y_component = { fabsf(body_down.axis.y), 1, body_down.axis.y };
    PoseAxisComponent z_component = { fabsf(body_down.axis.z), 2, body_down.axis.z };
    const char *primary_label;
    const char *secondary_label;

    if ((label == NULL) || (label_size == 0U))
    {
        return;
    }

    FullRangeAngles_SortDescending(&x_component, &y_component, &z_component);

    primary_label = FullRangeAngles_AxisLabel(x_component.axis, x_component.value);
    secondary_label = FullRangeAngles_AxisLabel(y_component.axis, y_component.value);

    if (x_component.magnitude < 0.45f)
    {
        (void)snprintf(label, label_size, "tilted");
        return;
    }

    if ((x_component.magnitude >= 0.65f) && (y_component.magnitude >= 0.55f))
    {
        (void)snprintf(label, label_size, "%s + %s", primary_label, secondary_label);
        return;
    }

    if ((x_component.magnitude - y_component.magnitude) >= 0.20f)
    {
        (void)snprintf(label, label_size, "%s", primary_label);
        return;
    }

    if (y_component.magnitude >= 0.45f)
    {
        (void)snprintf(label, label_size, "%s + %s", primary_label, secondary_label);
        return;
    }

    (void)snprintf(label, label_size, "tilted");
}