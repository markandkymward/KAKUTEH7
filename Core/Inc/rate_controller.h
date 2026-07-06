#ifndef RATE_CONTROLLER_H
#define RATE_CONTROLLER_H

#include <stdint.h>

typedef struct {
  float kp;
  float ki;
  float kd;
  float integrator;
  float previousError;
  float output;
  float outputLimit;
  float integratorLimit;
} RatePidAxis;

typedef struct {
  RatePidAxis roll;
  RatePidAxis pitch;
  RatePidAxis yaw;
} RateController;

void RateController_Init(RateController *controller);
void RateController_Reset(RateController *controller);
void RateController_SetGains(RateController *controller,
                             float rollP,
                             float rollI,
                             float rollD,
                             float pitchP,
                             float pitchI,
                             float pitchD,
                             float yawP,
                             float yawI,
                             float yawD);
void RateController_Update(RateController *controller,
                           float dt,
                           float desiredRollRateDps,
                           float desiredPitchRateDps,
                           float desiredYawRateDps,
                           float gyroRollRateDps,
                           float gyroPitchRateDps,
                           float gyroYawRateDps,
                           float *rollCmd,
                           float *pitchCmd,
                           float *yawCmd);

#endif /* RATE_CONTROLLER_H */
