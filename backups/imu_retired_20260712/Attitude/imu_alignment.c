#include "imu_alignment.h"

#include <stddef.h>

ImuAxis3f ImuAlignmentSensorToBody(ImuAxis3f sensor_axes)
{
    ImuAxis3f body_axes;

    body_axes.x = -sensor_axes.y;
    body_axes.y = -sensor_axes.x;
    body_axes.z = -sensor_axes.z;

    return body_axes;
}

void ImuAlignmentSensorToBodyAxes(float sensor_x,
                                  float sensor_y,
                                  float sensor_z,
                                  float *body_x,
                                  float *body_y,
                                  float *body_z)
{
    const ImuAxis3f body_axes = ImuAlignmentSensorToBody((ImuAxis3f){
        .x = sensor_x,
        .y = sensor_y,
        .z = sensor_z,
    });

    if (body_x != NULL)
    {
        *body_x = body_axes.x;
    }
    if (body_y != NULL)
    {
        *body_y = body_axes.y;
    }
    if (body_z != NULL)
    {
        *body_z = body_axes.z;
    }
}