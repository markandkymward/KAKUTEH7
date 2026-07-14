#ifndef IMU_ALIGNMENT_H
#define IMU_ALIGNMENT_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x;
    float y;
    float z;
} ImuAxis3f;

ImuAxis3f ImuAlignmentSensorToBody(ImuAxis3f sensor_axes);
void ImuAlignmentSensorToBodyAxes(float sensor_x,
                                  float sensor_y,
                                  float sensor_z,
                                  float *body_x,
                                  float *body_y,
                                  float *body_z);

#ifdef __cplusplus
}
#endif

#endif