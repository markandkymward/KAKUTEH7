#include "telemetry_link.h"

#include <string.h>

#include "command_rx.h"
#include "tm_protocol.h"

#define TELEMETRY_LINK_RX_RING_BYTES 512U

typedef struct {
  UART_HandleTypeDef *uart;
  volatile uint8_t rxIrqByte;
  volatile uint16_t rxHead;
  volatile uint16_t rxTail;
  uint8_t rxRing[TELEMETRY_LINK_RX_RING_BYTES];

  volatile uint8_t txBusy;
  uint8_t txActive[TM_PROTOCOL_MAX_FRAME_BYTES];
  uint16_t txActiveLen;

  uint8_t cmdPending[TM_PROTOCOL_MAX_FRAME_BYTES];
  uint16_t cmdPendingLen;
  uint8_t tmPending[TM_PROTOCOL_MAX_FRAME_BYTES];
  uint16_t tmPendingLen;

  uint32_t txSequence;

  TmFrameParser parser;
  CommandRxContext commandContext;
  TelemetryCounters counters;
} TelemetryLinkContext;

static TelemetryLinkContext g_tm_link;

static bool telemetry_link_ring_push(uint8_t byte)
{
  uint16_t nextHead = (uint16_t)((g_tm_link.rxHead + 1U) & (TELEMETRY_LINK_RX_RING_BYTES - 1U));

  if (nextHead == g_tm_link.rxTail)
  {
    g_tm_link.counters.cmdRxBadCrc++;
    return false;
  }

  g_tm_link.rxRing[g_tm_link.rxHead] = byte;
  g_tm_link.rxHead = nextHead;
  return true;
}

static bool telemetry_link_ring_pop(uint8_t *outByte)
{
  if ((outByte == NULL) || (g_tm_link.rxTail == g_tm_link.rxHead))
  {
    return false;
  }

  *outByte = g_tm_link.rxRing[g_tm_link.rxTail];
  g_tm_link.rxTail = (uint16_t)((g_tm_link.rxTail + 1U) & (TELEMETRY_LINK_RX_RING_BYTES - 1U));
  return true;
}

static void telemetry_link_sync_counters_from_command_context(void)
{
  g_tm_link.counters.cmdRxTotal = g_tm_link.commandContext.counters.rxTotal;
  g_tm_link.counters.cmdRxValid = g_tm_link.commandContext.counters.rxValid;
  g_tm_link.counters.cmdRxBadCrc = g_tm_link.commandContext.counters.rxBadCrc;
  g_tm_link.counters.cmdRxUnknown = g_tm_link.commandContext.counters.rxUnknown;
  g_tm_link.counters.ackSent = g_tm_link.commandContext.counters.ackSent;
  g_tm_link.counters.nackSent = g_tm_link.commandContext.counters.nackSent;
}

static void telemetry_link_try_kick_tx(void)
{
  uint8_t *src = NULL;
  uint16_t len = 0U;

  if ((g_tm_link.uart == NULL) || (g_tm_link.txBusy != 0U))
  {
    return;
  }

  if (g_tm_link.cmdPendingLen > 0U)
  {
    src = g_tm_link.cmdPending;
    len = g_tm_link.cmdPendingLen;
    g_tm_link.cmdPendingLen = 0U;
  }
  else if (g_tm_link.tmPendingLen > 0U)
  {
    src = g_tm_link.tmPending;
    len = g_tm_link.tmPendingLen;
    g_tm_link.tmPendingLen = 0U;
  }
  else
  {
    return;
  }

  if ((src == NULL) || (len == 0U) || (len > sizeof(g_tm_link.txActive)))
  {
    return;
  }

  memcpy(g_tm_link.txActive, src, len);
  g_tm_link.txActiveLen = len;

  if (HAL_UART_Transmit_IT(g_tm_link.uart, g_tm_link.txActive, len) == HAL_OK)
  {
    g_tm_link.txBusy = 1U;
    return;
  }

  /* If transmit start fails, keep command priority by restoring pending slot. */
  if (src == g_tm_link.cmdPending)
  {
    memcpy(g_tm_link.cmdPending, g_tm_link.txActive, len);
    g_tm_link.cmdPendingLen = len;
  }
  else
  {
    memcpy(g_tm_link.tmPending, g_tm_link.txActive, len);
    g_tm_link.tmPendingLen = len;
  }
}

static void telemetry_link_queue_command_reply(const CommandRxReply *reply)
{
  TelemetryCommandReply payload;
  uint8_t payloadBytes[16];
  uint8_t frame[TM_PROTOCOL_MAX_FRAME_BYTES];
  uint16_t payloadLen = 0U;
  uint16_t frameLen = 0U;

  if (reply == NULL)
  {
    return;
  }

  payload.commandType = reply->commandType;
  payload.reason = reply->nackReason;
  payload.statusCode = reply->statusCode;
  payload.commandSequence = reply->commandSequence;

  payloadLen = Telemetry_PackCommandReply(&payload, payloadBytes, sizeof(payloadBytes));
  if (payloadLen == 0U)
  {
    return;
  }

  frameLen = TmProtocol_EncodeFrame(TM_FLAG_DIR_FC_TO_HOST,
                                    (reply->replyType == COMMAND_RX_REPLY_ACK)
                                      ? (uint8_t)TM_PACKET_TYPE_TM_COMMAND_ACK
                                      : (uint8_t)TM_PACKET_TYPE_TM_COMMAND_NACK,
                                    payloadLen,
                                    g_tm_link.txSequence++,
                                    HAL_GetTick(),
                                    payloadBytes,
                                    frame,
                                    sizeof(frame));
  if (frameLen == 0U)
  {
    return;
  }

  memcpy(g_tm_link.cmdPending, frame, frameLen);
  g_tm_link.cmdPendingLen = frameLen;
  telemetry_link_try_kick_tx();
}

bool TelemetryLink_Init(UART_HandleTypeDef *huart)
{
  memset(&g_tm_link, 0, sizeof(g_tm_link));
  g_tm_link.uart = huart;
  g_tm_link.txSequence = 1U;

  TmProtocol_ParserInit(&g_tm_link.parser);
  CommandRx_Init(&g_tm_link.commandContext);
  telemetry_link_sync_counters_from_command_context();

  if (g_tm_link.uart == NULL)
  {
    return false;
  }

  if (HAL_UART_Receive_IT(g_tm_link.uart, (uint8_t *)&g_tm_link.rxIrqByte, 1U) != HAL_OK)
  {
    return false;
  }

  return true;
}

void TelemetryLink_Update(void)
{
  uint8_t byte = 0U;
  TmFrameView frameView;

  while (telemetry_link_ring_pop(&byte))
  {
    TmParserFeedResult result = TmProtocol_ParserPushByte(&g_tm_link.parser, byte, &frameView);

    if (result == TM_PARSER_FEED_CRC_ERROR)
    {
      CommandRx_OnParserCrcError(&g_tm_link.commandContext);
      continue;
    }

    if (result == TM_PARSER_FEED_FRAME_READY)
    {
      CommandRxReply reply;
      if (CommandRx_HandleFrame(&g_tm_link.commandContext, &frameView, &reply))
      {
        if (reply.replyType != COMMAND_RX_REPLY_NONE)
        {
          telemetry_link_queue_command_reply(&reply);
        }
      }
    }
  }

  telemetry_link_sync_counters_from_command_context();
  telemetry_link_try_kick_tx();
}

bool TelemetryLink_SendCombinedFast(const TelemetryCombinedFast *combined)
{
  TelemetryCombinedFast snapshot;
  uint8_t payload[TELEMETRY_COMBINED_FAST_PAYLOAD_BYTES];
  uint8_t frame[TM_PROTOCOL_MAX_FRAME_BYTES];
  uint16_t payloadLen = 0U;
  uint16_t frameLen = 0U;

  if (combined == NULL)
  {
    return false;
  }

  if (g_tm_link.uart == NULL)
  {
    return false;
  }

  if ((g_tm_link.txBusy != 0U) || (g_tm_link.tmPendingLen != 0U))
  {
    g_tm_link.counters.tmTxDroppedBusy++;
    return false;
  }

  snapshot = *combined;
  snapshot.telemetryPacketCounter = g_tm_link.counters.tmTxTotal;
  snapshot.commandPacketCounter = g_tm_link.counters.cmdRxValid;
  snapshot.droppedPacketCounter = g_tm_link.counters.tmTxDroppedBusy;
  snapshot.crcErrorCounter = g_tm_link.counters.cmdRxBadCrc;
  snapshot.unknownCommandCounter = g_tm_link.counters.cmdRxUnknown;

  payloadLen = Telemetry_PackCombinedFast(&snapshot, payload, sizeof(payload));
  if (payloadLen == 0U)
  {
    return false;
  }

  frameLen = TmProtocol_EncodeFrame(TM_FLAG_DIR_FC_TO_HOST,
                                    (uint8_t)TM_PACKET_TYPE_TM_COMBINED_FAST,
                                    payloadLen,
                                    g_tm_link.txSequence++,
                                    HAL_GetTick(),
                                    payload,
                                    frame,
                                    sizeof(frame));
  if (frameLen == 0U)
  {
    return false;
  }

  memcpy(g_tm_link.tmPending, frame, frameLen);
  g_tm_link.tmPendingLen = frameLen;
  g_tm_link.counters.tmTxTotal++;
  telemetry_link_try_kick_tx();
  return true;
}

void TelemetryLink_GetCounters(TelemetryCounters *outCounters)
{
  if (outCounters == NULL)
  {
    return;
  }

  telemetry_link_sync_counters_from_command_context();
  *outCounters = g_tm_link.counters;
}

void TelemetryLink_OnUartRxCplt(UART_HandleTypeDef *huart)
{
  if ((huart == NULL) || (huart != g_tm_link.uart))
  {
    return;
  }

  (void)telemetry_link_ring_push(g_tm_link.rxIrqByte);
  (void)HAL_UART_Receive_IT(g_tm_link.uart, (uint8_t *)&g_tm_link.rxIrqByte, 1U);
}

void TelemetryLink_OnUartTxCplt(UART_HandleTypeDef *huart)
{
  if ((huart == NULL) || (huart != g_tm_link.uart))
  {
    return;
  }

  g_tm_link.txBusy = 0U;
  g_tm_link.txActiveLen = 0U;
  telemetry_link_try_kick_tx();
}

void TelemetryLink_OnUartError(UART_HandleTypeDef *huart)
{
  if ((huart == NULL) || (huart != g_tm_link.uart))
  {
    return;
  }

  g_tm_link.txBusy = 0U;
  g_tm_link.txActiveLen = 0U;
  (void)HAL_UART_AbortReceive(huart);
  (void)HAL_UART_Receive_IT(g_tm_link.uart, (uint8_t *)&g_tm_link.rxIrqByte, 1U);
}
