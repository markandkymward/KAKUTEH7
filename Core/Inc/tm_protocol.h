#ifndef TM_PROTOCOL_H
#define TM_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define TM_SYNC1 0xA5U
#define TM_SYNC2 0x5AU
#define TM_VERSION 0x01U

#define TM_FLAG_DIR_FC_TO_HOST 0x01U
#define TM_FLAG_DIR_HOST_TO_FC 0x00U

#define TM_PROTOCOL_MAX_PAYLOAD_BYTES 192U
#define TM_PROTOCOL_HEADER_BYTES 14U
#define TM_PROTOCOL_CRC_BYTES 2U
#define TM_PROTOCOL_FRAME_OVERHEAD (TM_PROTOCOL_HEADER_BYTES + TM_PROTOCOL_CRC_BYTES)
#define TM_PROTOCOL_MAX_FRAME_BYTES (TM_PROTOCOL_MAX_PAYLOAD_BYTES + TM_PROTOCOL_FRAME_OVERHEAD)

typedef enum {
  TM_PACKET_TYPE_TM_STATUS = 0x01,
  TM_PACKET_TYPE_TM_GYRO = 0x02,
  TM_PACKET_TYPE_TM_ATTITUDE = 0x03,
  TM_PACKET_TYPE_TM_RC = 0x04,
  TM_PACKET_TYPE_TM_MOTORS = 0x05,
  TM_PACKET_TYPE_TM_COMBINED_FAST = 0x06,
  TM_PACKET_TYPE_TM_COMMAND_ACK = 0x07,
  TM_PACKET_TYPE_TM_COMMAND_NACK = 0x08,

  TM_PACKET_TYPE_CMD_PING = 0x80,
  TM_PACKET_TYPE_CMD_GET_VERSION = 0x81,
  TM_PACKET_TYPE_CMD_STREAM_ENABLE = 0x82,
  TM_PACKET_TYPE_CMD_SET_STREAM_RATE = 0x83,
  TM_PACKET_TYPE_CMD_SET_DISPLAY_FILTER_CUTOFF = 0x84,
  TM_PACKET_TYPE_CMD_SET_DEBUG_FILTER_CUTOFF = 0x85,
  TM_PACKET_TYPE_CMD_GET_STATUS = 0x86,
  TM_PACKET_TYPE_CMD_GET_RATES = 0x87,
  TM_PACKET_TYPE_CMD_SET_RATES_STAGED = 0x88,
  TM_PACKET_TYPE_CMD_APPLY_STAGED_RATES = 0x89,
  TM_PACKET_TYPE_CMD_SAVE_SETTINGS_REQUEST = 0x8A
} TmPacketType;

typedef enum {
  TM_NACK_REASON_NONE = 0,
  TM_NACK_REASON_BAD_CRC = 1,
  TM_NACK_REASON_UNKNOWN_CMD = 2,
  TM_NACK_REASON_BAD_LENGTH = 3,
  TM_NACK_REASON_BUSY = 4,
  TM_NACK_REASON_FORBIDDEN_ARMED = 5,
  TM_NACK_REASON_OUT_OF_RANGE = 6
} TmNackReason;

typedef struct {
  uint8_t version;
  uint8_t flags;
  uint8_t packetType;
  uint16_t payloadLength;
  uint32_t sequence;
  uint32_t timestampMs;
  const uint8_t *payload;
  uint16_t crc16;
} TmFrameView;

typedef enum {
  TM_PARSER_FEED_NONE = 0,
  TM_PARSER_FEED_FRAME_READY,
  TM_PARSER_FEED_CRC_ERROR,
  TM_PARSER_FEED_OVERFLOW
} TmParserFeedResult;

typedef struct {
  uint8_t state;
  uint16_t index;
  uint16_t expectedPayloadLength;
  uint8_t buffer[TM_PROTOCOL_MAX_FRAME_BYTES];
} TmFrameParser;

uint16_t TmProtocol_Crc16Ccitt(const uint8_t *data, uint16_t length);

bool TmProtocol_DecodeFrame(const uint8_t *frameBytes,
                            uint16_t frameLength,
                            TmFrameView *outFrame);

uint16_t TmProtocol_EncodeFrame(uint8_t flags,
                                uint8_t packetType,
                                uint16_t payloadLength,
                                uint32_t sequence,
                                uint32_t timestampMs,
                                const uint8_t *payload,
                                uint8_t *outFrame,
                                uint16_t outCapacity);

void TmProtocol_ParserInit(TmFrameParser *parser);

TmParserFeedResult TmProtocol_ParserPushByte(TmFrameParser *parser,
                                             uint8_t byte,
                                             TmFrameView *outFrame);

#endif
