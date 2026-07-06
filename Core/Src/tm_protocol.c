#include "tm_protocol.h"

#include <string.h>

#define TM_PARSE_STATE_SYNC1 0U
#define TM_PARSE_STATE_SYNC2 1U
#define TM_PARSE_STATE_FIXED_HEADER 2U
#define TM_PARSE_STATE_PAYLOAD_AND_CRC 3U

static uint16_t tm_read_u16_le(const uint8_t *src)
{
  return (uint16_t)src[0] | ((uint16_t)src[1] << 8U);
}

static uint32_t tm_read_u32_le(const uint8_t *src)
{
  return (uint32_t)src[0]
       | ((uint32_t)src[1] << 8U)
       | ((uint32_t)src[2] << 16U)
       | ((uint32_t)src[3] << 24U);
}

static void tm_write_u16_le(uint8_t *dst, uint16_t value)
{
  dst[0] = (uint8_t)(value & 0xFFU);
  dst[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static void tm_write_u32_le(uint8_t *dst, uint32_t value)
{
  dst[0] = (uint8_t)(value & 0xFFU);
  dst[1] = (uint8_t)((value >> 8U) & 0xFFU);
  dst[2] = (uint8_t)((value >> 16U) & 0xFFU);
  dst[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

uint16_t TmProtocol_Crc16Ccitt(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFFU;

  if (data == NULL)
  {
    return crc;
  }

  for (uint16_t i = 0U; i < length; i++)
  {
    crc ^= ((uint16_t)data[i] << 8U);
    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
      if ((crc & 0x8000U) != 0U)
      {
        crc = (uint16_t)((crc << 1U) ^ 0x1021U);
      }
      else
      {
        crc <<= 1U;
      }
    }
  }

  return crc;
}

bool TmProtocol_DecodeFrame(const uint8_t *frameBytes,
                            uint16_t frameLength,
                            TmFrameView *outFrame)
{
  uint16_t payloadLength = 0U;
  uint16_t computedCrc = 0U;
  uint16_t receivedCrc = 0U;

  if ((frameBytes == NULL) || (outFrame == NULL))
  {
    return false;
  }

  if (frameLength < TM_PROTOCOL_FRAME_OVERHEAD)
  {
    return false;
  }

  if ((frameBytes[0] != TM_SYNC1) || (frameBytes[1] != TM_SYNC2))
  {
    return false;
  }

  payloadLength = tm_read_u16_le(&frameBytes[5]);
  if (payloadLength > TM_PROTOCOL_MAX_PAYLOAD_BYTES)
  {
    return false;
  }

  if (frameLength != (uint16_t)(TM_PROTOCOL_FRAME_OVERHEAD + payloadLength))
  {
    return false;
  }

  receivedCrc = tm_read_u16_le(&frameBytes[TM_PROTOCOL_HEADER_BYTES + payloadLength]);
  computedCrc = TmProtocol_Crc16Ccitt(&frameBytes[2],
                                      (uint16_t)(TM_PROTOCOL_HEADER_BYTES - 2U + payloadLength));
  if (computedCrc != receivedCrc)
  {
    return false;
  }

  outFrame->version = frameBytes[2];
  outFrame->flags = frameBytes[3];
  outFrame->packetType = frameBytes[4];
  outFrame->payloadLength = payloadLength;
  outFrame->sequence = tm_read_u32_le(&frameBytes[7]);
  outFrame->timestampMs = tm_read_u32_le(&frameBytes[11]);
  outFrame->payload = &frameBytes[TM_PROTOCOL_HEADER_BYTES];
  outFrame->crc16 = receivedCrc;

  return true;
}

uint16_t TmProtocol_EncodeFrame(uint8_t flags,
                                uint8_t packetType,
                                uint16_t payloadLength,
                                uint32_t sequence,
                                uint32_t timestampMs,
                                const uint8_t *payload,
                                uint8_t *outFrame,
                                uint16_t outCapacity)
{
  uint16_t totalLength = (uint16_t)(TM_PROTOCOL_FRAME_OVERHEAD + payloadLength);
  uint16_t crc = 0U;

  if (outFrame == NULL)
  {
    return 0U;
  }

  if (payloadLength > TM_PROTOCOL_MAX_PAYLOAD_BYTES)
  {
    return 0U;
  }

  if ((payloadLength > 0U) && (payload == NULL))
  {
    return 0U;
  }

  if (outCapacity < totalLength)
  {
    return 0U;
  }

  outFrame[0] = TM_SYNC1;
  outFrame[1] = TM_SYNC2;
  outFrame[2] = TM_VERSION;
  outFrame[3] = flags;
  outFrame[4] = packetType;
  tm_write_u16_le(&outFrame[5], payloadLength);
  tm_write_u32_le(&outFrame[7], sequence);
  tm_write_u32_le(&outFrame[11], timestampMs);

  if (payloadLength > 0U)
  {
    memcpy(&outFrame[TM_PROTOCOL_HEADER_BYTES], payload, payloadLength);
  }

  crc = TmProtocol_Crc16Ccitt(&outFrame[2],
                              (uint16_t)(TM_PROTOCOL_HEADER_BYTES - 2U + payloadLength));
  tm_write_u16_le(&outFrame[TM_PROTOCOL_HEADER_BYTES + payloadLength], crc);

  return totalLength;
}

void TmProtocol_ParserInit(TmFrameParser *parser)
{
  if (parser == NULL)
  {
    return;
  }

  parser->state = TM_PARSE_STATE_SYNC1;
  parser->index = 0U;
  parser->expectedPayloadLength = 0U;
  memset(parser->buffer, 0, sizeof(parser->buffer));
}

TmParserFeedResult TmProtocol_ParserPushByte(TmFrameParser *parser,
                                             uint8_t byte,
                                             TmFrameView *outFrame)
{
  uint16_t targetLength = 0U;

  if (parser == NULL)
  {
    return TM_PARSER_FEED_NONE;
  }

  switch (parser->state)
  {
    case TM_PARSE_STATE_SYNC1:
      if (byte == TM_SYNC1)
      {
        parser->buffer[0] = byte;
        parser->index = 1U;
        parser->state = TM_PARSE_STATE_SYNC2;
      }
      return TM_PARSER_FEED_NONE;

    case TM_PARSE_STATE_SYNC2:
      if (byte == TM_SYNC2)
      {
        parser->buffer[1] = byte;
        parser->index = 2U;
        parser->state = TM_PARSE_STATE_FIXED_HEADER;
      }
      else if (byte == TM_SYNC1)
      {
        parser->buffer[0] = byte;
        parser->index = 1U;
      }
      else
      {
        parser->state = TM_PARSE_STATE_SYNC1;
        parser->index = 0U;
      }
      return TM_PARSER_FEED_NONE;

    case TM_PARSE_STATE_FIXED_HEADER:
      parser->buffer[parser->index++] = byte;
      if (parser->index >= TM_PROTOCOL_HEADER_BYTES)
      {
        parser->expectedPayloadLength = tm_read_u16_le(&parser->buffer[5]);
        if (parser->expectedPayloadLength > TM_PROTOCOL_MAX_PAYLOAD_BYTES)
        {
          TmProtocol_ParserInit(parser);
          return TM_PARSER_FEED_OVERFLOW;
        }
        parser->state = TM_PARSE_STATE_PAYLOAD_AND_CRC;
      }
      return TM_PARSER_FEED_NONE;

    case TM_PARSE_STATE_PAYLOAD_AND_CRC:
      if (parser->index >= TM_PROTOCOL_MAX_FRAME_BYTES)
      {
        TmProtocol_ParserInit(parser);
        return TM_PARSER_FEED_OVERFLOW;
      }

      parser->buffer[parser->index++] = byte;
      targetLength = (uint16_t)(TM_PROTOCOL_HEADER_BYTES + parser->expectedPayloadLength + TM_PROTOCOL_CRC_BYTES);
      if (parser->index >= targetLength)
      {
        bool ok = TmProtocol_DecodeFrame(parser->buffer, parser->index, outFrame);
        TmProtocol_ParserInit(parser);
        return ok ? TM_PARSER_FEED_FRAME_READY : TM_PARSER_FEED_CRC_ERROR;
      }
      return TM_PARSER_FEED_NONE;

    default:
      TmProtocol_ParserInit(parser);
      return TM_PARSER_FEED_NONE;
  }
}
