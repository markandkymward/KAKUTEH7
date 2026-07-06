#include "rate_controller.h"

#include <stddef.h>

#define RATE_CONTROLLER_DT_MIN_SEC 0.0005f
#define RATE_CONTROLLER_DT_MAX_SEC 0.02f
#define RATE_CONTROLLER_DEFAULT_ROLL_KP 0.002f
#define RATE_CONTROLLER_DEFAULT_PITCH_KP 0.002f
#define RATE_CONTROLLER_DEFAULT_YAW_KP 0.001f
#define RATE_CONTROLLER_DEFAULT_KI 0.0f
#define RATE_CONTROLLER_DEFAULT_KD 0.0f
#define RATE_CONTROLLER_DEFAULT_OUTPUT_LIMIT 0.35f
#define RATE_CONTROLLER_DEFAULT_INTEGRATOR_LIMIT 0.20f

static float rate_controller_clamp(float value, float min_value, float max_value)
{
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return value;
}

static void rate_pid_axis_init(RatePidAxis *axis, float kp, float ki, float kd)
{
  if (axis == NULL)
  {
    return;
  }

  axis->kp = kp;
  axis->ki = ki;
  axis->kd = kd;
  axis->integrator = 0.0f;
  axis->previousError = 0.0f;
  axis->output = 0.0f;
  axis->outputLimit = RATE_CONTROLLER_DEFAULT_OUTPUT_LIMIT;
  axis->integratorLimit = RATE_CONTROLLER_DEFAULT_INTEGRATOR_LIMIT;
}

static float rate_pid_axis_update(RatePidAxis *axis, float dt, float desired, float measured)
{
  float error;
  float derivative;
  float output;

  if (axis == NULL)
  {
    return 0.0f;
  }

  error = desired - measured;

  axis->integrator += error * dt;
  axis->integrator = rate_controller_clamp(axis->integrator,
                                           -axis->integratorLimit,
                                           axis->integratorLimit);

  derivative = (error - axis->previousError) / dt;
  axis->previousError = error;

  output = (axis->kp * error)
         + (axis->ki * axis->integrator)
         + (axis->kd * derivative);

  output = rate_controller_clamp(output,
                                 -axis->outputLimit,
                                 axis->outputLimit);

  axis->output = output;
  return output;
}

void RateController_Init(RateController *controller)
{
  if (controller == NULL)
  {
    return;
  }

  rate_pid_axis_init(&controller->roll,
                     RATE_CONTROLLER_DEFAULT_ROLL_KP,
                     RATE_CONTROLLER_DEFAULT_KI,
                     RATE_CONTROLLER_DEFAULT_KD);
  rate_pid_axis_init(&controller->pitch,
                     RATE_CONTROLLER_DEFAULT_PITCH_KP,
                     RATE_CONTROLLER_DEFAULT_KI,
                     RATE_CONTROLLER_DEFAULT_KD);
  rate_pid_axis_init(&controller->yaw,
                     RATE_CONTROLLER_DEFAULT_YAW_KP,
                     RATE_CONTROLLER_DEFAULT_KI,
                     RATE_CONTROLLER_DEFAULT_KD);
}

void RateController_Reset(RateController *controller)
{
  if (controller == NULL)
  {
    return;
  }

  controller->roll.integrator = 0.0f;
  controller->roll.previousError = 0.0f;
  controller->roll.output = 0.0f;

  controller->pitch.integrator = 0.0f;
  controller->pitch.previousError = 0.0f;
  controller->pitch.output = 0.0f;

  controller->yaw.integrator = 0.0f;
  controller->yaw.previousError = 0.0f;
  controller->yaw.output = 0.0f;
}

void RateController_SetGains(RateController *controller,
                             float rollP,
                             float rollI,
                             float rollD,
                             float pitchP,
                             float pitchI,
                             float pitchD,
                             float yawP,
                             float yawI,
                             float yawD)
{
  if (controller == NULL)
  {
    return;
  }

  controller->roll.kp = rollP;
  controller->roll.ki = rollI;
  controller->roll.kd = rollD;

  controller->pitch.kp = pitchP;
  controller->pitch.ki = pitchI;
  controller->pitch.kd = pitchD;

  controller->yaw.kp = yawP;
  controller->yaw.ki = yawI;
  controller->yaw.kd = yawD;
}

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
                           float *yawCmd)
{
  float clamped_dt;

  if (controller == NULL)
  {
    return;
  }

  clamped_dt = rate_controller_clamp(dt,
                                     RATE_CONTROLLER_DT_MIN_SEC,
                                     RATE_CONTROLLER_DT_MAX_SEC);

  if (rollCmd != NULL)
  {
    *rollCmd = rate_pid_axis_update(&controller->roll,
                                    clamped_dt,
                                    desiredRollRateDps,
                                    gyroRollRateDps);
  }
  else
  {
    (void)rate_pid_axis_update(&controller->roll,
                               clamped_dt,
                               desiredRollRateDps,
                               gyroRollRateDps);
  }

  if (pitchCmd != NULL)
  {
    *pitchCmd = rate_pid_axis_update(&controller->pitch,
                                     clamped_dt,
                                     desiredPitchRateDps,
                                     gyroPitchRateDps);
  }
  else
  {
    (void)rate_pid_axis_update(&controller->pitch,
                               clamped_dt,
                               desiredPitchRateDps,
                               gyroPitchRateDps);
  }

  if (yawCmd != NULL)
  {
    *yawCmd = rate_pid_axis_update(&controller->yaw,
                                   clamped_dt,
                                   desiredYawRateDps,
                                   gyroYawRateDps);
  }
  else
  {
    (void)rate_pid_axis_update(&controller->yaw,
                               clamped_dt,
                               desiredYawRateDps,
                               gyroYawRateDps);
  }
}
