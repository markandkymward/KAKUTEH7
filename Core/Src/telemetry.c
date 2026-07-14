#include "telemetry.h"

#include <string.h>

_Static_assert(TELEMETRY_COMBINED_FAST_PAYLOAD_BYTES == 80U,
               "TELEMETRY_COMBINED_FAST_PAYLOAD_BYTES must be 80 bytes");
_Static_assert(sizeof(TelemetryCombinedFast) == TELEMETRY_COMBINED_FAST_PAYLOAD_BYTES,
               "TelemetryCombinedFast struct size must match TM payload size");

#define TELEMETRY_COMMAND_REPLY_PAYLOAD_BYTES 8U

static void telemetry_put_u16_le(uint8_t *dst, uint16_t value)
{
  dst[0] = (uint8_t)(value & 0xFFU);
  dst[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static void telemetry_put_u32_le(uint8_t *dst, uint32_t value)
{
  dst[0] = (uint8_t)(value & 0xFFU);
  dst[1] = (uint8_t)((value >> 8U) & 0xFFU);
  dst[2] = (uint8_t)((value >> 16U) & 0xFFU);
  dst[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static void telemetry_put_s16_le(uint8_t *dst, int16_t value)
{
  telemetry_put_u16_le(dst, (uint16_t)value);
}

static uint16_t telemetry_read_u16_le(const uint8_t *src)
{
  return (uint16_t)src[0] | ((uint16_t)src[1] << 8U);
}

static uint32_t telemetry_read_u32_le(const uint8_t *src)
{
  return (uint32_t)src[0]
       | ((uint32_t)src[1] << 8U)
       | ((uint32_t)src[2] << 16U)
       | ((uint32_t)src[3] << 24U);
}

static int16_t telemetry_read_s16_le(const uint8_t *src)
{
  return (int16_t)telemetry_read_u16_le(src);
}

uint16_t Telemetry_PackCombinedFast(const TelemetryCombinedFast *src,
                                    uint8_t *dst,
                                    uint16_t dstCapacity)
{
  uint16_t index = 0U;

  if ((src == NULL) || (dst == NULL) || (dstCapacity < TELEMETRY_COMBINED_FAST_PAYLOAD_BYTES))
  {
    return 0U;
  }

  telemetry_put_s16_le(&dst[index], src->gyroXdps10); index += 2U;
  telemetry_put_s16_le(&dst[index], src->gyroYdps10); index += 2U;
  telemetry_put_s16_le(&dst[index], src->gyroZdps10); index += 2U;
  telemetry_put_s16_le(&dst[index], src->sensorGyroXraw); index += 2U;
  telemetry_put_s16_le(&dst[index], src->sensorGyroYraw); index += 2U;
  telemetry_put_s16_le(&dst[index], src->sensorGyroZraw); index += 2U;
  telemetry_put_s16_le(&dst[index], src->bodyGyroXraw); index += 2U;
  telemetry_put_s16_le(&dst[index], src->bodyGyroYraw); index += 2U;
  telemetry_put_s16_le(&dst[index], src->bodyGyroZraw); index += 2U;
  telemetry_put_s16_le(&dst[index], src->accelXraw); index += 2U;
  telemetry_put_s16_le(&dst[index], src->accelYraw); index += 2U;
  telemetry_put_s16_le(&dst[index], src->accelZraw); index += 2U;
  telemetry_put_s16_le(&dst[index], src->rollCd); index += 2U;
  telemetry_put_s16_le(&dst[index], src->pitchCd); index += 2U;
  telemetry_put_s16_le(&dst[index], src->yawCd); index += 2U;
  telemetry_put_s16_le(&dst[index], src->q0x10000); index += 2U;
  telemetry_put_s16_le(&dst[index], src->q1x10000); index += 2U;
  telemetry_put_s16_le(&dst[index], src->q2x10000); index += 2U;
  telemetry_put_s16_le(&dst[index], src->q3x10000); index += 2U;

  telemetry_put_u16_le(&dst[index], src->rcThrottleUs); index += 2U;
  telemetry_put_u16_le(&dst[index], src->rcRollUs); index += 2U;
  telemetry_put_u16_le(&dst[index], src->rcPitchUs); index += 2U;
  telemetry_put_u16_le(&dst[index], src->rcYawUs); index += 2U;
  telemetry_put_u16_le(&dst[index], src->rcArmUs); index += 2U;

  telemetry_put_u16_le(&dst[index], src->motor1Us); index += 2U;
  telemetry_put_u16_le(&dst[index], src->motor2Us); index += 2U;
  telemetry_put_u16_le(&dst[index], src->motor3Us); index += 2U;
  telemetry_put_u16_le(&dst[index], src->motor4Us); index += 2U;

  telemetry_put_u16_le(&dst[index], src->loopDtUs); index += 2U;
  dst[index++] = src->armed;
  dst[index++] = src->failsafeFlags;

  telemetry_put_u32_le(&dst[index], src->telemetryPacketCounter); index += 4U;
  telemetry_put_u32_le(&dst[index], src->commandPacketCounter); index += 4U;
  telemetry_put_u32_le(&dst[index], src->droppedPacketCounter); index += 4U;
  telemetry_put_u32_le(&dst[index], src->crcErrorCounter); index += 4U;
  telemetry_put_u32_le(&dst[index], src->unknownCommandCounter); index += 4U;

  return index;
}

bool Telemetry_UnpackCombinedFast(const uint8_t *src,
                                  uint16_t srcLength,
                                  TelemetryCombinedFast *dst)
{
  uint16_t index = 0U;

  if ((src == NULL) || (dst == NULL) || (srcLength != TELEMETRY_COMBINED_FAST_PAYLOAD_BYTES))
  {
    return false;
  }

  memset(dst, 0, sizeof(*dst));

  dst->gyroXdps10 = telemetry_read_s16_le(&src[index]); index += 2U;
  dst->gyroYdps10 = telemetry_read_s16_le(&src[index]); index += 2U;
  dst->gyroZdps10 = telemetry_read_s16_le(&src[index]); index += 2U;
  dst->sensorGyroXraw = telemetry_read_s16_le(&src[index]); index += 2U;
  dst->sensorGyroYraw = telemetry_read_s16_le(&src[index]); index += 2U;
  dst->sensorGyroZraw = telemetry_read_s16_le(&src[index]); index += 2U;
  dst->bodyGyroXraw = telemetry_read_s16_le(&src[index]); index += 2U;
  dst->bodyGyroYraw = telemetry_read_s16_le(&src[index]); index += 2U;
  dst->bodyGyroZraw = telemetry_read_s16_le(&src[index]); index += 2U;
  dst->accelXraw = telemetry_read_s16_le(&src[index]); index += 2U;
  dst->accelYraw = telemetry_read_s16_le(&src[index]); index += 2U;
  dst->accelZraw = telemetry_read_s16_le(&src[index]); index += 2U;
  dst->rollCd = telemetry_read_s16_le(&src[index]); index += 2U;
  dst->pitchCd = telemetry_read_s16_le(&src[index]); index += 2U;
  dst->yawCd = telemetry_read_s16_le(&src[index]); index += 2U;
  dst->q0x10000 = telemetry_read_s16_le(&src[index]); index += 2U;
  dst->q1x10000 = telemetry_read_s16_le(&src[index]); index += 2U;
  dst->q2x10000 = telemetry_read_s16_le(&src[index]); index += 2U;
  dst->q3x10000 = telemetry_read_s16_le(&src[index]); index += 2U;

  dst->rcThrottleUs = telemetry_read_u16_le(&src[index]); index += 2U;
  dst->rcRollUs = telemetry_read_u16_le(&src[index]); index += 2U;
  dst->rcPitchUs = telemetry_read_u16_le(&src[index]); index += 2U;
  dst->rcYawUs = telemetry_read_u16_le(&src[index]); index += 2U;
  dst->rcArmUs = telemetry_read_u16_le(&src[index]); index += 2U;

  dst->motor1Us = telemetry_read_u16_le(&src[index]); index += 2U;
  dst->motor2Us = telemetry_read_u16_le(&src[index]); index += 2U;
  dst->motor3Us = telemetry_read_u16_le(&src[index]); index += 2U;
  dst->motor4Us = telemetry_read_u16_le(&src[index]); index += 2U;

  dst->loopDtUs = telemetry_read_u16_le(&src[index]); index += 2U;
  dst->armed = src[index++];
  dst->failsafeFlags = src[index++];

  dst->telemetryPacketCounter = telemetry_read_u32_le(&src[index]); index += 4U;
  dst->commandPacketCounter = telemetry_read_u32_le(&src[index]); index += 4U;
  dst->droppedPacketCounter = telemetry_read_u32_le(&src[index]); index += 4U;
  dst->crcErrorCounter = telemetry_read_u32_le(&src[index]); index += 4U;
  dst->unknownCommandCounter = telemetry_read_u32_le(&src[index]); index += 4U;

  return (index == TELEMETRY_COMBINED_FAST_PAYLOAD_BYTES);
}

uint16_t Telemetry_PackCommandReply(const TelemetryCommandReply *src,
                                    uint8_t *dst,
                                    uint16_t dstCapacity)
{
  if ((src == NULL) || (dst == NULL) || (dstCapacity < TELEMETRY_COMMAND_REPLY_PAYLOAD_BYTES))
  {
    return 0U;
  }

  dst[0] = src->commandType;
  dst[1] = src->reason;
  telemetry_put_u16_le(&dst[2], src->statusCode);
  telemetry_put_u32_le(&dst[4], src->commandSequence);
  return TELEMETRY_COMMAND_REPLY_PAYLOAD_BYTES;
}

bool Telemetry_UnpackCommandReply(const uint8_t *src,
                                  uint16_t srcLength,
                                  TelemetryCommandReply *dst)
{
  if ((src == NULL) || (dst == NULL) || (srcLength != TELEMETRY_COMMAND_REPLY_PAYLOAD_BYTES))
  {
    return false;
  }

  dst->commandType = src[0];
  dst->reason = src[1];
  dst->statusCode = telemetry_read_u16_le(&src[2]);
  dst->commandSequence = telemetry_read_u32_le(&src[4]);
  return true;
}
