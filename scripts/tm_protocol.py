from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Optional

TM_SYNC1 = 0xA5
TM_SYNC2 = 0x5A
TM_VERSION = 0x01

TM_FLAG_DIR_HOST_TO_FC = 0x00
TM_FLAG_DIR_FC_TO_HOST = 0x01

TM_PROTOCOL_MAX_PAYLOAD = 192
TM_HEADER_BYTES = 14
TM_CRC_BYTES = 2


class PacketType(IntEnum):
    TM_STATUS = 0x01
    TM_GYRO = 0x02
    TM_ATTITUDE = 0x03
    TM_RC = 0x04
    TM_MOTORS = 0x05
    TM_COMBINED_FAST = 0x06
    TM_COMMAND_ACK = 0x07
    TM_COMMAND_NACK = 0x08

    CMD_PING = 0x80
    CMD_GET_VERSION = 0x81
    CMD_STREAM_ENABLE = 0x82
    CMD_SET_STREAM_RATE = 0x83
    CMD_SET_DISPLAY_FILTER_CUTOFF = 0x84
    CMD_SET_DEBUG_FILTER_CUTOFF = 0x85
    CMD_GET_STATUS = 0x86
    CMD_GET_RATES = 0x87
    CMD_SET_RATES_STAGED = 0x88
    CMD_APPLY_STAGED_RATES = 0x89
    CMD_SAVE_SETTINGS_REQUEST = 0x8A


class NackReason(IntEnum):
    NONE = 0
    BAD_CRC = 1
    UNKNOWN_CMD = 2
    BAD_LENGTH = 3
    BUSY = 4
    FORBIDDEN_ARMED = 5
    OUT_OF_RANGE = 6


@dataclass
class Frame:
    version: int
    flags: int
    packet_type: int
    payload_length: int
    sequence: int
    timestamp_ms: int
    payload: bytes
    crc16: int


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def encode_frame(
    flags: int,
    packet_type: int,
    sequence: int,
    timestamp_ms: int,
    payload: bytes,
) -> bytes:
    if len(payload) > TM_PROTOCOL_MAX_PAYLOAD:
        raise ValueError("payload too large")

    header = bytearray()
    header.append(TM_SYNC1)
    header.append(TM_SYNC2)
    header.append(TM_VERSION)
    header.append(flags & 0xFF)
    header.append(packet_type & 0xFF)
    header.extend(len(payload).to_bytes(2, "little"))
    header.extend((sequence & 0xFFFFFFFF).to_bytes(4, "little"))
    header.extend((timestamp_ms & 0xFFFFFFFF).to_bytes(4, "little"))

    crc = crc16_ccitt(bytes(header[2:]) + payload)
    return bytes(header) + payload + crc.to_bytes(2, "little")


def decode_frame(frame: bytes) -> Frame:
    if len(frame) < (TM_HEADER_BYTES + TM_CRC_BYTES):
        raise ValueError("frame too short")
    if frame[0] != TM_SYNC1 or frame[1] != TM_SYNC2:
        raise ValueError("bad sync")

    payload_length = int.from_bytes(frame[5:7], "little")
    expected = TM_HEADER_BYTES + payload_length + TM_CRC_BYTES
    if len(frame) != expected:
        raise ValueError("bad length")

    received_crc = int.from_bytes(frame[-2:], "little")
    computed_crc = crc16_ccitt(frame[2:-2])
    if received_crc != computed_crc:
        raise ValueError("bad crc")

    payload = frame[TM_HEADER_BYTES:-2]
    return Frame(
        version=frame[2],
        flags=frame[3],
        packet_type=frame[4],
        payload_length=payload_length,
        sequence=int.from_bytes(frame[7:11], "little"),
        timestamp_ms=int.from_bytes(frame[11:15], "little"),
        payload=payload,
        crc16=received_crc,
    )


class StreamParser:
    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> list[Frame]:
        self._buf.extend(data)
        out: list[Frame] = []

        while True:
            start = self._buf.find(bytes((TM_SYNC1, TM_SYNC2)))
            if start < 0:
                self._buf[:] = self._buf[-1:] if (self._buf and self._buf[-1] == TM_SYNC1) else b""
                break

            if start > 0:
                del self._buf[:start]

            if len(self._buf) < TM_HEADER_BYTES + TM_CRC_BYTES:
                break

            payload_length = int.from_bytes(self._buf[5:7], "little")
            if payload_length > TM_PROTOCOL_MAX_PAYLOAD:
                del self._buf[0]
                continue

            total = TM_HEADER_BYTES + payload_length + TM_CRC_BYTES
            if len(self._buf) < total:
                break

            frame_bytes = bytes(self._buf[:total])
            del self._buf[:total]

            try:
                out.append(decode_frame(frame_bytes))
            except ValueError:
                continue

        return out


def pack_cmd_ping(sequence: int, timestamp_ms: int, nonce: Optional[int] = None) -> bytes:
    payload = b""
    if nonce is not None:
        payload = (nonce & 0xFFFFFFFF).to_bytes(4, "little")
    return encode_frame(
        flags=TM_FLAG_DIR_HOST_TO_FC,
        packet_type=PacketType.CMD_PING,
        sequence=sequence,
        timestamp_ms=timestamp_ms,
        payload=payload,
    )
