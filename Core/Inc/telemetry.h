#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#include "tm_protocol.h"

#define TELEMETRY_COMBINED_FAST_PAYLOAD_BYTES 54U

typedef struct {
  uint32_t cmdRxTotal;
  uint32_t cmdRxValid;
  uint32_t cmdRxBadCrc;
  uint32_t cmdRxUnknown;
  uint32_t ackSent;
  uint32_t nackSent;
  uint32_t tmTxTotal;
  uint32_t tmTxDroppedBusy;
} TelemetryCounters;

typedef struct {
  int16_t gyroXdps10;
  int16_t gyroYdps10;
  int16_t gyroZdps10;
  int16_t rollCd;
  int16_t pitchCd;
  int16_t yawCd;
  uint16_t rcThrottleUs;
  uint16_t rcRollUs;
  uint16_t rcPitchUs;
  uint16_t rcYawUs;
  uint16_t rcArmUs;
  uint16_t motor1Us;
  uint16_t motor2Us;
  uint16_t motor3Us;
  uint16_t motor4Us;
  uint16_t loopDtUs;
  uint8_t armed;
  uint8_t failsafeFlags;
  uint32_t telemetryPacketCounter;
  uint32_t commandPacketCounter;
  uint32_t droppedPacketCounter;
  uint32_t crcErrorCounter;
  uint32_t unknownCommandCounter;
} TelemetryCombinedFast;

typedef struct {
  uint8_t commandType;
  uint8_t reason;
  uint16_t statusCode;
  uint32_t commandSequence;
} TelemetryCommandReply;

uint16_t Telemetry_PackCombinedFast(const TelemetryCombinedFast *src,
                                    uint8_t *dst,
                                    uint16_t dstCapacity);

bool Telemetry_UnpackCombinedFast(const uint8_t *src,
                                  uint16_t srcLength,
                                  TelemetryCombinedFast *dst);

uint16_t Telemetry_PackCommandReply(const TelemetryCommandReply *src,
                                    uint8_t *dst,
                                    uint16_t dstCapacity);

bool Telemetry_UnpackCommandReply(const uint8_t *src,
                                  uint16_t srcLength,
                                  TelemetryCommandReply *dst);

#endif
