#!/usr/bin/env python3
"""Read line-based telemetry from the KAKUTEH7 serial stream.

Usage:
  python scripts/read_serial.py --port COM9

Requires:
  pip install pyserial
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from typing import Iterable

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover - host dependency only
    raise SystemExit(
        "pyserial is required. Install it with: pip install pyserial"
    ) from exc


def _list_ports() -> Iterable[str]:
    for port in list_ports.comports():
        yield f"{port.device}: {port.description}"


def _pick_port() -> str | None:
    preferred_keywords = ("Betaflight", "STM32H743", "USB Serial Device", "CDC")
    fallback = None

    for port in list_ports.comports():
        description = f"{port.device}: {port.description}"
        if "STLink" in port.description or "ST-LINK" in port.description:
            continue
        if any(keyword.lower() in description.lower() for keyword in preferred_keywords):
            return port.device
        if fallback is None:
            fallback = port.device

    return fallback

SOF1 = 0xA5
SOF2 = 0x5A
PACKET_TYPE_TELEMETRY = 0x01
TELEMETRY_PAYLOAD_LEN = 28
TELEMETRY_STRUCT = struct.Struct("<Iiii6h")


def _checksum(packet_type: int, payload: bytes) -> int:
    result = packet_type ^ len(payload)
    for byte in payload:
        result ^= byte
    return result & 0xFF


def _format_telemetry(values: tuple[int, int, int, int, int, int, int, int, int, int]) -> str:
    sequence, pitch_cd, roll_cd, yaw_cd, gx, gy, gz, ax, ay, az = values
    return (
        f"seq={sequence:06d}  "
        f"P:{pitch_cd / 100.0:>7.2f}  R:{roll_cd / 100.0:>7.2f}  Y:{yaw_cd / 100.0:>7.2f}  "
        f"GX:{gx:>6}  GY:{gy:>6}  GZ:{gz:>6}  AX:{ax:>6}  AY:{ay:>6}  AZ:{az:>6}"
    )


def _decode_frames(buffer: bytearray) -> Iterable[str]:
    while True:
        start = buffer.find(bytes((SOF1, SOF2)))
        if start < 0:
            if buffer:
                # Keep a tiny tail in case SOF1/SOF2 straddles a read boundary.
                buffer[:] = buffer[-1:] if buffer[-1] == SOF1 else b""
            return

        if start > 0:
            del buffer[:start]

        if len(buffer) < 4:
            return

        packet_type = buffer[2]
        payload_len = buffer[3]
        frame_len = 4 + payload_len + 1

        if len(buffer) < frame_len:
            return

        payload = bytes(buffer[4:4 + payload_len])
        checksum = buffer[4 + payload_len]
        del buffer[:frame_len]

        if checksum != _checksum(packet_type, payload):
            continue

        if packet_type != PACKET_TYPE_TELEMETRY or payload_len != TELEMETRY_PAYLOAD_LEN:
            continue

        yield _format_telemetry(TELEMETRY_STRUCT.unpack(payload))


def main() -> int:
    parser = argparse.ArgumentParser(description="Read telemetry from a serial COM port.")
    parser.add_argument("--port", help="Serial port, for example COM6 or 'auto'", default="COM6")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    args = parser.parse_args()

    if args.port.lower() == "auto":
        args.port = _pick_port()

    if not args.port:
        ports = list(_list_ports())
        if not ports:
            print("No serial ports found.", file=sys.stderr)
        else:
            print("Available serial ports:", file=sys.stderr)
            for port in ports:
                print(f"  {port}", file=sys.stderr)
        print("\nExample: python scripts/read_serial.py --port COM9", file=sys.stderr)
        return 1

    try:
        with serial.Serial(args.port, args.baud, timeout=1) as ser:
            print(f"Listening on {args.port} at {args.baud} baud...", file=sys.stderr)
            buffer = bytearray()
            last_line = ""
            last_packet_time = time.monotonic()
            while True:
                chunk = ser.read(ser.in_waiting or 1)
                if not chunk:
                    if time.monotonic() - last_packet_time >= 5.0:
                        print(f"WAIT {args.port} no packets yet", file=sys.stderr)
                        last_packet_time = time.monotonic()
                    continue
                buffer.extend(chunk)
                saw_packet = False
                for rendered in _decode_frames(buffer):
                    padding = max(0, len(last_line) - len(rendered))
                    sys.stdout.write("\r" + rendered + (" " * padding))
                    sys.stdout.flush()
                    last_line = rendered
                    saw_packet = True
                    last_packet_time = time.monotonic()
                if not saw_packet and time.monotonic() - last_packet_time >= 5.0:
                    print(f"WAIT {args.port} receiving bytes but no valid packets yet", file=sys.stderr)
                    last_packet_time = time.monotonic()
    except serial.SerialException as exc:
        print(f"Could not open {args.port}: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
