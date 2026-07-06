#ifndef COMMAND_RX_H
#define COMMAND_RX_H

#include <stdbool.h>
#include <stdint.h>

#include "tm_protocol.h"

typedef struct {
  uint32_t rxTotal;
  uint32_t rxValid;
  uint32_t rxBadCrc;
  uint32_t rxUnknown;
  uint32_t ackSent;
  uint32_t nackSent;
} CommandRxCounters;

typedef struct {
  bool streamEnabled;
  uint16_t streamRateHz;
  uint32_t lastPingNonce;
  uint32_t lastCommandSequence;
  CommandRxCounters counters;
} CommandRxContext;

typedef enum {
  COMMAND_RX_REPLY_NONE = 0,
  COMMAND_RX_REPLY_ACK,
  COMMAND_RX_REPLY_NACK
} CommandRxReplyType;

typedef struct {
  CommandRxReplyType replyType;
  uint8_t commandType;
  uint8_t nackReason;
  uint16_t statusCode;
  uint32_t commandSequence;
} CommandRxReply;

void CommandRx_Init(CommandRxContext *ctx);

void CommandRx_OnParserCrcError(CommandRxContext *ctx);

bool CommandRx_HandleFrame(CommandRxContext *ctx,
                           const TmFrameView *frame,
                           CommandRxReply *reply);

#endif
