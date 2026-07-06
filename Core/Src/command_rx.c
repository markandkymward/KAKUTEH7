#include "command_rx.h"

#include <string.h>

void CommandRx_Init(CommandRxContext *ctx)
{
  if (ctx == NULL)
  {
    return;
  }

  memset(ctx, 0, sizeof(*ctx));
  ctx->streamEnabled = true;
  ctx->streamRateHz = 50U;
}

void CommandRx_OnParserCrcError(CommandRxContext *ctx)
{
  if (ctx == NULL)
  {
    return;
  }

  ctx->counters.rxBadCrc++;
}

static void command_rx_make_ack(CommandRxContext *ctx,
                                const TmFrameView *frame,
                                CommandRxReply *reply)
{
  if ((ctx == NULL) || (frame == NULL) || (reply == NULL))
  {
    return;
  }

  reply->replyType = COMMAND_RX_REPLY_ACK;
  reply->commandType = frame->packetType;
  reply->nackReason = TM_NACK_REASON_NONE;
  reply->statusCode = 0U;
  reply->commandSequence = frame->sequence;

  ctx->counters.rxValid++;
  ctx->counters.ackSent++;
  ctx->lastCommandSequence = frame->sequence;
}

static void command_rx_make_nack(CommandRxContext *ctx,
                                 const TmFrameView *frame,
                                 CommandRxReply *reply,
                                 uint8_t reason,
                                 uint16_t status)
{
  if ((ctx == NULL) || (frame == NULL) || (reply == NULL))
  {
    return;
  }

  reply->replyType = COMMAND_RX_REPLY_NACK;
  reply->commandType = frame->packetType;
  reply->nackReason = reason;
  reply->statusCode = status;
  reply->commandSequence = frame->sequence;

  ctx->counters.nackSent++;
}

bool CommandRx_HandleFrame(CommandRxContext *ctx,
                           const TmFrameView *frame,
                           CommandRxReply *reply)
{
  if ((ctx == NULL) || (frame == NULL) || (reply == NULL))
  {
    return false;
  }

  memset(reply, 0, sizeof(*reply));
  ctx->counters.rxTotal++;

  if (frame->version != TM_VERSION)
  {
    command_rx_make_nack(ctx,
                         frame,
                         reply,
                         TM_NACK_REASON_BAD_LENGTH,
                         0x0001U);
    return true;
  }

  if ((frame->flags & TM_FLAG_DIR_FC_TO_HOST) != 0U)
  {
    command_rx_make_nack(ctx,
                         frame,
                         reply,
                         TM_NACK_REASON_UNKNOWN_CMD,
                         0x0002U);
    return true;
  }

  switch (frame->packetType)
  {
    case TM_PACKET_TYPE_CMD_PING:
      if (frame->payloadLength == 4U)
      {
        ctx->lastPingNonce = (uint32_t)frame->payload[0]
                           | ((uint32_t)frame->payload[1] << 8U)
                           | ((uint32_t)frame->payload[2] << 16U)
                           | ((uint32_t)frame->payload[3] << 24U);
      }
      else if (frame->payloadLength != 0U)
      {
        command_rx_make_nack(ctx,
                             frame,
                             reply,
                             TM_NACK_REASON_BAD_LENGTH,
                             0x0101U);
        return true;
      }

      command_rx_make_ack(ctx, frame, reply);
      return true;

    default:
      ctx->counters.rxUnknown++;
      command_rx_make_nack(ctx,
                           frame,
                           reply,
                           TM_NACK_REASON_UNKNOWN_CMD,
                           0xFFFFU);
      return true;
  }
}
