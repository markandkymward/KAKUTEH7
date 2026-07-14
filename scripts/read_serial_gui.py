#!/usr/bin/env python3
"""Graphical telemetry viewer for KAKUTEH7 serial stream.

Usage:
	python scripts/read_serial_gui.py --port COM6

Requires:
  pip install pyserial
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import os
import socket
import struct
import sys
import threading
import time
import tkinter as tk
from collections import deque
from dataclasses import dataclass, field
from tkinter import ttk
from typing import Iterable

try:
	import serial
	from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover
	raise SystemExit("pyserial is required. Install it with: pip install pyserial") from exc

try:
	_tm_protocol_path = os.path.join(os.path.dirname(__file__), "tm_protocol.py")
	_tm_protocol_spec = importlib.util.spec_from_file_location("kakute_tm_protocol", _tm_protocol_path)
	if (_tm_protocol_spec is None) or (_tm_protocol_spec.loader is None):
		raise ImportError("failed to load local tm_protocol.py")
	tm_protocol = importlib.util.module_from_spec(_tm_protocol_spec)
	sys.modules[_tm_protocol_spec.name] = tm_protocol
	_tm_protocol_spec.loader.exec_module(tm_protocol)
except Exception:  # pragma: no cover
	tm_protocol = None


SOF1 = 0xA5
SOF2 = 0x5A
PACKET_TYPE_TELEMETRY = 0x01
PACKET_TYPE_MOTOR_TEST = 0x10
PACKET_TYPE_DISPLAY_FILTER = 0x11
TELEMETRY_PAYLOAD_LEN_V1 = 73
TELEMETRY_PAYLOAD_LEN_V2 = 83
TELEMETRY_PAYLOAD_LEN_V3 = 84
TELEMETRY_PAYLOAD_LEN_V4 = 92
TELEMETRY_PAYLOAD_LEN_V5 = 94
TELEMETRY_PAYLOAD_LEN_V6 = 98
TELEMETRY_PAYLOAD_LEN_V7 = 104
TELEMETRY_PAYLOAD_LEN_V8 = 116
TELEMETRY_PAYLOAD_LEN_V9 = 122
TELEMETRY_PAYLOAD_LEN_V10 = 130
TELEMETRY_STRUCT_V1 = struct.Struct("<Iiii6hBIIi16H")
TELEMETRY_STRUCT_V2 = struct.Struct("<Iiii6hBIIi16HBB4H")
TELEMETRY_STRUCT_V3 = struct.Struct("<Iiii6hBIIi16HBBB4H")
TELEMETRY_STRUCT_V4 = struct.Struct("<Iiii9hBIIi16HBBB5H")
TELEMETRY_STRUCT_V5 = struct.Struct("<Iiii9hBIIiH16HBBB5H")
TELEMETRY_STRUCT_V6 = struct.Struct("<Iiii9hBIIiH16HBBB5HHBB")
TELEMETRY_STRUCT_V7 = struct.Struct("<Iiii9hBIIiH16HBBB5HHBBhhh")
TELEMETRY_STRUCT_V8 = struct.Struct("<Iiii9hBIIiH16HBBB5HHBBhhhhhhhhh")
TELEMETRY_STRUCT_V9 = struct.Struct("<Iiii9hBIIiH16HBBB5HHBBhhhhhhhhhhhh")
TELEMETRY_STRUCT_V10 = struct.Struct("<Iiii9hBIIiH16HBBB5HHBBhhhhhhhhhhhhhhhh")
IMU_ACCEL_LSB_PER_G = 2048.0
IMU_GYRO_DPS_PER_LSB = 2000.0 / 32768.0
LAYOUT_SCHEMA_VERSION = 3
TM_COMBINED_FAST_STRUCT = struct.Struct("<hhhhhhhhhhhhhhhhhhhHHHHHHHHHHBBIIIII")
TM_COMBINED_FAST_STRUCT_WITH_ACCEL = struct.Struct("<hhhhhhhhhhhhhHHHHHHHHHHBBIIIII")
TM_COMBINED_FAST_STRUCT_WITH_QUAT = struct.Struct("<hhhhhhhhhhHHHHHHHHHHBBIIIII")
TM_COMBINED_FAST_STRUCT_LEGACY = struct.Struct("<hhhhhhHHHHHHHHHHBBIIIII")

GUI_UPDATE_INTERVAL_MS = 16
SERIAL_READ_TIMEOUT_S = 0.03
SOCKET_READ_TIMEOUT_S = 0.05
PERF_MODE_NORMAL = "Normal"
PERF_MODE_LOW_LATENCY = "Low-latency"
GUI_BUILD_TAG = "GUI-TMLOCK-1"
SERIAL_NO_FRAME_ROTATE_S = 3.0


def _checksum(packet_type: int, payload: bytes) -> int:
	result = packet_type ^ len(payload)
	for byte in payload:
		result ^= byte
	return result & 0xFF


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


def _parse_tcp_endpoint(port_spec: str) -> tuple[str, int] | None:
	text = (port_spec or "").strip()
	if not text or text.lower() == "auto":
		return None
	if text.upper().startswith("COM"):
		return None
	if ":" not in text:
		return None

	host, port_text = text.rsplit(":", 1)
	host = host.strip()
	port_text = port_text.strip()
	if not host or not port_text.isdigit():
		return None

	port = int(port_text)
	if port < 1 or port > 65535:
		return None

	return host, port


def _decode_frames(buffer: bytearray) -> Iterable[tuple[int, ...]]:
	while True:
		start = buffer.find(bytes((SOF1, SOF2)))
		if start < 0:
			if buffer:
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

		payload = bytes(buffer[4 : 4 + payload_len])
		checksum = buffer[4 + payload_len]
		del buffer[:frame_len]

		if checksum != _checksum(packet_type, payload):
			continue
		if packet_type != PACKET_TYPE_TELEMETRY:
			continue
		if payload_len == TELEMETRY_PAYLOAD_LEN_V1:
			unpacked = TELEMETRY_STRUCT_V1.unpack(payload)
			yield unpacked + (0, 0, 0, 1000, 1000, 1000, 1000)
			continue
		if payload_len == TELEMETRY_PAYLOAD_LEN_V2:
			unpacked = TELEMETRY_STRUCT_V2.unpack(payload)
			yield unpacked[:32] + (0,) + unpacked[32:]
			continue
		if payload_len == TELEMETRY_PAYLOAD_LEN_V3:
			yield TELEMETRY_STRUCT_V3.unpack(payload)
			continue
		if payload_len == TELEMETRY_PAYLOAD_LEN_V4:
			yield TELEMETRY_STRUCT_V4.unpack(payload)
			continue
		if payload_len == TELEMETRY_PAYLOAD_LEN_V5:
			yield TELEMETRY_STRUCT_V5.unpack(payload)
			continue
		if payload_len == TELEMETRY_PAYLOAD_LEN_V6:
			yield TELEMETRY_STRUCT_V6.unpack(payload)
			continue
		if payload_len == TELEMETRY_PAYLOAD_LEN_V7:
			yield TELEMETRY_STRUCT_V7.unpack(payload)
			continue
		if payload_len == TELEMETRY_PAYLOAD_LEN_V8:
			yield TELEMETRY_STRUCT_V8.unpack(payload)
			continue
		if payload_len == TELEMETRY_PAYLOAD_LEN_V9:
			yield TELEMETRY_STRUCT_V9.unpack(payload)
			continue
		if payload_len == TELEMETRY_PAYLOAD_LEN_V10:
			yield TELEMETRY_STRUCT_V10.unpack(payload)
			continue


def _build_motor_test_packet(motor_index: int, motor_us: int) -> bytes:
	payload = bytes(
		(
			max(1, min(4, int(motor_index))),
			int(max(1000, min(2000, int(motor_us)))) & 0xFF,
			(int(max(1000, min(2000, int(motor_us)))) >> 8) & 0xFF,
		)
	)
	packet_type = PACKET_TYPE_MOTOR_TEST
	frame = bytearray((SOF1, SOF2, packet_type, len(payload)))
	frame.extend(payload)
	frame.append(_checksum(packet_type, payload))
	return bytes(frame)


def _build_motor_test_ascii(motor_index: int, motor_us: int) -> bytes:
	idx = max(1, min(4, int(motor_index)))
	us = max(1000, min(2000, int(motor_us)))
	return f"MTEST {idx} {us}\n".encode("ascii")


def _build_display_filter_packet(cutoff_hz: int) -> bytes:
	cutoff = max(0, min(10, int(cutoff_hz)))
	payload = bytes((cutoff & 0xFF, (cutoff >> 8) & 0xFF))
	packet_type = PACKET_TYPE_DISPLAY_FILTER
	frame = bytearray((SOF1, SOF2, packet_type, len(payload)))
	frame.extend(payload)
	frame.append(_checksum(packet_type, payload))
	return bytes(frame)


def _build_display_filter_ascii(cutoff_hz: int) -> bytes:
	cutoff = max(0, min(10, int(cutoff_hz)))
	return f"DFLT {cutoff}\n".encode("ascii")


@dataclass
class TelemetryState:
	sequence: int = 0
	pitch_deg: float = 0.0
	roll_deg: float = 0.0
	yaw_deg: float = 0.0
	fw_quaternion: tuple[float, float, float, float] | None = None
	gx: int = 0
	gy: int = 0
	gz: int = 0
	gx_filtered: int = 0
	gy_filtered: int = 0
	gz_filtered: int = 0
	ax: int = 0
	ay: int = 0
	az: int = 0
	display_filter_cutoff_hz: int = 5
	pressure_pa: int = 0
	altitude_cm: int = 0
	battery_mv: int | None = None
	battery_voltage_v: float | None = None
	battery_adc_raw: int = 0
	battery_adc_channel: int = 255
	battery_adc_ready: int = 0
	desired_roll_rate_dps: float = 0.0
	desired_pitch_rate_dps: float = 0.0
	desired_yaw_rate_dps: float = 0.0
	gyro_roll_rate_dps: float = 0.0
	gyro_pitch_rate_dps: float = 0.0
	gyro_yaw_rate_dps: float = 0.0
	roll_rate_error_dps: float = 0.0
	pitch_rate_error_dps: float = 0.0
	yaw_rate_error_dps: float = 0.0
	roll_pid_output: float = 0.0
	pitch_pid_output: float = 0.0
	yaw_pid_output: float = 0.0
	rc_link_ok: bool = False
	rc_frames: int = 0
	channels: list[int] = field(default_factory=lambda: [0] * 16)
	motor_mode: int = 0
	motors_armed: int = 0
	gui_test_active: int = 0
	gyro_filter_available: bool = False
	motors_us: list[int] = field(default_factory=lambda: [1000, 1000, 1000, 1000])
	last_update_monotonic: float = 0.0
	connected: bool = False
	port_name: str = ""
	status_text: str = "Sniffing serial ports..."
	tm_profile: str = "TM?"
	tm_payload_len: int = 0


class SerialReader(threading.Thread):
	def __init__(self, port: str, baud: int, state: TelemetryState, lock: threading.Lock):
		super().__init__(daemon=True)
		self.port = port
		self.baud = baud
		self.tcp_endpoint = _parse_tcp_endpoint(port)
		self.state = state
		self.lock = lock
		self.running = True
		self.error_message = ""
		self._tx_lock = threading.Lock()
		self._tx_queue: list[bytes] = []
		self._tcp_stream_state = "unknown"
		self._last_good_serial_port = ""

	@staticmethod
	def _score_serial_port(device: str, description: str, last_good_port: str) -> int:
		text = f"{device} {description}".lower()
		score = 0
		if device.upper() == last_good_port.upper() and last_good_port:
			score += 100
		if "betaflight" in text or "stm32h743" in text:
			score += 60
		if "cdc" in text or "usb serial device" in text:
			score += 30
		if "camera" in text:
			score -= 80
		if "ch343" in text or "arduino" in text:
			score -= 20
		return score

	def stop(self) -> None:
		self.running = False

	def queue_command(self, frame: bytes) -> None:
		if not frame:
			return
		with self._tx_lock:
			self._tx_queue.append(frame)

	def _drain_tx_serial(self, ser: serial.Serial) -> None:
		with self._tx_lock:
			pending = self._tx_queue
			self._tx_queue = []
		for frame in pending:
			try:
				ser.write(frame)
			except Exception:
				self.error_message = "Failed to send command frame"
				return

	def _drain_tx_socket(self, sock: socket.socket) -> None:
		with self._tx_lock:
			pending = self._tx_queue
			self._tx_queue = []
		for frame in pending:
			try:
				sock.sendall(frame)
			except Exception:
				self.error_message = "Failed to send command frame"
				return

	def _apply_decoded(self, candidate: str, values: tuple[int, ...]) -> None:
		if len(values) == 61:
			(
				sequence,
				_pitch_cd,
				_roll_cd,
				_yaw_cd,
				gx,
				gy,
				gz,
				gx_filtered,
				gy_filtered,
				gz_filtered,
				ax,
				ay,
				az,
				rc_link,
				rc_frames,
				pressure_pa,
				altitude_cm,
				battery_mv,
				*channels,
			) = values
			display_filter_cutoff_hz = int(channels[23])
			battery_adc_raw = int(channels[24])
			battery_adc_channel = int(channels[25])
			battery_adc_ready = int(channels[26])
			desired_roll_rate_dps = int(channels[27]) * 0.1
			desired_pitch_rate_dps = int(channels[28]) * 0.1
			desired_yaw_rate_dps = int(channels[29]) * 0.1
			gyro_roll_rate_dps = int(channels[30]) * 0.1
			gyro_pitch_rate_dps = int(channels[31]) * 0.1
			gyro_yaw_rate_dps = int(channels[32]) * 0.1
			roll_rate_error_dps = int(channels[33]) * 0.1
			pitch_rate_error_dps = int(channels[34]) * 0.1
			yaw_rate_error_dps = int(channels[35]) * 0.1
			roll_pid_output = int(channels[36]) * 0.0001
			pitch_pid_output = int(channels[37]) * 0.0001
			yaw_pid_output = int(channels[38]) * 0.0001
			battery_mv = int(battery_mv)
			battery_voltage_v = float(battery_mv) * 0.001
			gyro_filter_available = True
		elif len(values) == 57:
			(
				sequence,
				_pitch_cd,
				_roll_cd,
				_yaw_cd,
				gx,
				gy,
				gz,
				gx_filtered,
				gy_filtered,
				gz_filtered,
				ax,
				ay,
				az,
				rc_link,
				rc_frames,
				pressure_pa,
				altitude_cm,
				battery_mv,
				*channels,
			) = values
			display_filter_cutoff_hz = int(channels[23])
			battery_adc_raw = int(channels[24])
			battery_adc_channel = int(channels[25])
			battery_adc_ready = int(channels[26])
			desired_roll_rate_dps = int(channels[27]) * 0.1
			desired_pitch_rate_dps = int(channels[28]) * 0.1
			desired_yaw_rate_dps = int(channels[29]) * 0.1
			gyro_roll_rate_dps = int(channels[30]) * 0.1
			gyro_pitch_rate_dps = int(channels[31]) * 0.1
			gyro_yaw_rate_dps = int(channels[32]) * 0.1
			roll_rate_error_dps = int(channels[33]) * 0.1
			pitch_rate_error_dps = int(channels[34]) * 0.1
			yaw_rate_error_dps = int(channels[35]) * 0.1
			roll_pid_output = int(channels[36]) * 0.0001
			pitch_pid_output = int(channels[37]) * 0.0001
			yaw_pid_output = int(channels[38]) * 0.0001
			battery_mv = int(battery_mv)
			battery_voltage_v = float(battery_mv) * 0.001
			gyro_filter_available = True
		elif len(values) == 54:
			(
				sequence,
				_pitch_cd,
				_roll_cd,
				_yaw_cd,
				gx,
				gy,
				gz,
				gx_filtered,
				gy_filtered,
				gz_filtered,
				ax,
				ay,
				az,
				rc_link,
				rc_frames,
				pressure_pa,
				altitude_cm,
				battery_mv,
				*channels,
			) = values
			display_filter_cutoff_hz = int(channels[23])
			battery_adc_raw = int(channels[24])
			battery_adc_channel = int(channels[25])
			battery_adc_ready = int(channels[26])
			desired_roll_rate_dps = int(channels[27]) * 0.1
			desired_pitch_rate_dps = int(channels[28]) * 0.1
			desired_yaw_rate_dps = int(channels[29]) * 0.1
			gyro_roll_rate_dps = int(channels[30]) * 0.1
			gyro_pitch_rate_dps = int(channels[31]) * 0.1
			gyro_yaw_rate_dps = int(channels[32]) * 0.1
			roll_rate_error_dps = int(channels[33]) * 0.1
			pitch_rate_error_dps = int(channels[34]) * 0.1
			yaw_rate_error_dps = int(channels[35]) * 0.1
			roll_pid_output = 0.0
			pitch_pid_output = 0.0
			yaw_pid_output = 0.0
			battery_mv = int(battery_mv)
			battery_voltage_v = float(battery_mv) * 0.001
			gyro_filter_available = True
		elif len(values) == 48:
			(
				sequence,
				_pitch_cd,
				_roll_cd,
				_yaw_cd,
				gx,
				gy,
				gz,
				gx_filtered,
				gy_filtered,
				gz_filtered,
				ax,
				ay,
				az,
				rc_link,
				rc_frames,
				pressure_pa,
				altitude_cm,
				battery_mv,
				*channels,
			) = values
			display_filter_cutoff_hz = int(channels[23])
			battery_adc_raw = int(channels[24])
			battery_adc_channel = int(channels[25])
			battery_adc_ready = int(channels[26])
			desired_roll_rate_dps = int(channels[27]) * 0.1
			desired_pitch_rate_dps = int(channels[28]) * 0.1
			desired_yaw_rate_dps = int(channels[29]) * 0.1
			gyro_roll_rate_dps = 0.0
			gyro_pitch_rate_dps = 0.0
			gyro_yaw_rate_dps = 0.0
			roll_rate_error_dps = 0.0
			pitch_rate_error_dps = 0.0
			yaw_rate_error_dps = 0.0
			roll_pid_output = 0.0
			pitch_pid_output = 0.0
			yaw_pid_output = 0.0
			battery_mv = int(battery_mv)
			battery_voltage_v = float(battery_mv) * 0.001
			gyro_filter_available = True
		elif len(values) == 45:
			(
				sequence,
				_pitch_cd,
				_roll_cd,
				_yaw_cd,
				gx,
				gy,
				gz,
				gx_filtered,
				gy_filtered,
				gz_filtered,
				ax,
				ay,
				az,
				rc_link,
				rc_frames,
				pressure_pa,
				altitude_cm,
				battery_mv,
				*channels,
			) = values
			display_filter_cutoff_hz = int(channels[23])
			battery_adc_raw = int(channels[24])
			battery_adc_channel = int(channels[25])
			battery_adc_ready = int(channels[26])
			desired_roll_rate_dps = 0.0
			desired_pitch_rate_dps = 0.0
			desired_yaw_rate_dps = 0.0
			gyro_roll_rate_dps = 0.0
			gyro_pitch_rate_dps = 0.0
			gyro_yaw_rate_dps = 0.0
			roll_rate_error_dps = 0.0
			pitch_rate_error_dps = 0.0
			yaw_rate_error_dps = 0.0
			roll_pid_output = 0.0
			pitch_pid_output = 0.0
			yaw_pid_output = 0.0
			battery_mv = int(battery_mv)
			battery_voltage_v = float(battery_mv) * 0.001
			gyro_filter_available = True
		elif len(values) == 42:
			(
				sequence,
				_pitch_cd,
				_roll_cd,
				_yaw_cd,
				gx,
				gy,
				gz,
				gx_filtered,
				gy_filtered,
				gz_filtered,
				ax,
				ay,
				az,
				rc_link,
				rc_frames,
				pressure_pa,
				altitude_cm,
				battery_mv,
				*channels,
			) = values
			display_filter_cutoff_hz = int(channels[23])
			battery_adc_raw = 0
			battery_adc_channel = 255
			battery_adc_ready = 0
			desired_roll_rate_dps = 0.0
			desired_pitch_rate_dps = 0.0
			desired_yaw_rate_dps = 0.0
			battery_mv = int(battery_mv)
			battery_voltage_v = float(battery_mv) * 0.001
			gyro_filter_available = True
			gyro_roll_rate_dps = 0.0
			gyro_pitch_rate_dps = 0.0
			gyro_yaw_rate_dps = 0.0
			roll_rate_error_dps = 0.0
			pitch_rate_error_dps = 0.0
			yaw_rate_error_dps = 0.0
			roll_pid_output = 0.0
			pitch_pid_output = 0.0
			yaw_pid_output = 0.0
		elif len(values) == 41:
			(
				sequence,
				_pitch_cd,
				_roll_cd,
				_yaw_cd,
				gx,
				gy,
				gz,
				gx_filtered,
				gy_filtered,
				gz_filtered,
				ax,
				ay,
				az,
				rc_link,
				rc_frames,
				pressure_pa,
				altitude_cm,
				*channels,
			) = values
			display_filter_cutoff_hz = int(channels[23])
			battery_adc_raw = 0
			battery_adc_channel = 255
			battery_adc_ready = 0
			desired_roll_rate_dps = 0.0
			desired_pitch_rate_dps = 0.0
			desired_yaw_rate_dps = 0.0
			gyro_roll_rate_dps = 0.0
			gyro_pitch_rate_dps = 0.0
			gyro_yaw_rate_dps = 0.0
			roll_rate_error_dps = 0.0
			pitch_rate_error_dps = 0.0
			yaw_rate_error_dps = 0.0
			roll_pid_output = 0.0
			pitch_pid_output = 0.0
			yaw_pid_output = 0.0
			battery_mv = None
			battery_voltage_v = None
			gyro_filter_available = True
		else:
			(
				sequence,
				_pitch_cd,
				_roll_cd,
				_yaw_cd,
				gx,
				gy,
				gz,
				ax,
				ay,
				az,
				rc_link,
				rc_frames,
				pressure_pa,
				altitude_cm,
				*channels,
			) = values
			gx_filtered = gx
			gy_filtered = gy
			gz_filtered = gz
			display_filter_cutoff_hz = 5
			battery_adc_raw = 0
			battery_adc_channel = 255
			battery_adc_ready = 0
			desired_roll_rate_dps = 0.0
			desired_pitch_rate_dps = 0.0
			desired_yaw_rate_dps = 0.0
			gyro_roll_rate_dps = 0.0
			gyro_pitch_rate_dps = 0.0
			gyro_yaw_rate_dps = 0.0
			roll_rate_error_dps = 0.0
			pitch_rate_error_dps = 0.0
			yaw_rate_error_dps = 0.0
			roll_pid_output = 0.0
			pitch_pid_output = 0.0
			yaw_pid_output = 0.0
			battery_mv = None
			battery_voltage_v = None
			gyro_filter_available = False

		motor_mode = int(channels[16])
		motors_armed = int(channels[17])
		gui_test_active = int(channels[18])
		motors_us = [int(v) for v in channels[19:23]]
		channels = channels[:16]

		with self.lock:
			self.state.pitch_deg = float(_pitch_cd) / 100.0
			self.state.roll_deg = float(_roll_cd) / 100.0
			self.state.yaw_deg = float(_yaw_cd) / 100.0
			self.state.sequence = int(sequence)
			self.state.gx = int(gx)
			self.state.gy = int(gy)
			self.state.gz = int(gz)
			self.state.gx_filtered = int(gx_filtered)
			self.state.gy_filtered = int(gy_filtered)
			self.state.gz_filtered = int(gz_filtered)
			self.state.ax = int(ax)
			self.state.ay = int(ay)
			self.state.az = int(az)
			self.state.display_filter_cutoff_hz = int(display_filter_cutoff_hz)
			self.state.pressure_pa = int(pressure_pa)
			self.state.altitude_cm = int(altitude_cm)
			self.state.battery_mv = battery_mv
			self.state.battery_voltage_v = battery_voltage_v
			self.state.battery_adc_raw = int(battery_adc_raw)
			self.state.battery_adc_channel = int(battery_adc_channel)
			self.state.battery_adc_ready = int(battery_adc_ready)
			self.state.desired_roll_rate_dps = float(desired_roll_rate_dps)
			self.state.desired_pitch_rate_dps = float(desired_pitch_rate_dps)
			self.state.desired_yaw_rate_dps = float(desired_yaw_rate_dps)
			self.state.gyro_roll_rate_dps = float(gyro_roll_rate_dps)
			self.state.gyro_pitch_rate_dps = float(gyro_pitch_rate_dps)
			self.state.gyro_yaw_rate_dps = float(gyro_yaw_rate_dps)
			self.state.roll_rate_error_dps = float(roll_rate_error_dps)
			self.state.pitch_rate_error_dps = float(pitch_rate_error_dps)
			self.state.yaw_rate_error_dps = float(yaw_rate_error_dps)
			self.state.roll_pid_output = float(roll_pid_output)
			self.state.pitch_pid_output = float(pitch_pid_output)
			self.state.yaw_pid_output = float(yaw_pid_output)
			self.state.rc_link_ok = bool(rc_link)
			self.state.rc_frames = int(rc_frames)
			self.state.channels = [int(v) for v in channels]
			self.state.motor_mode = motor_mode
			self.state.motors_armed = motors_armed
			self.state.gui_test_active = gui_test_active
			self.state.gyro_filter_available = bool(gyro_filter_available)
			self.state.motors_us = motors_us
			self.state.last_update_monotonic = time.monotonic()
			self.state.connected = True
			self.state.status_text = f"Connected: {candidate}"

	def _apply_tm_combined_fast(self, candidate: str, sequence: int, payload: bytes) -> None:
		if len(payload) not in (
			TM_COMBINED_FAST_STRUCT.size,
			TM_COMBINED_FAST_STRUCT_WITH_ACCEL.size,
			TM_COMBINED_FAST_STRUCT_WITH_QUAT.size,
			TM_COMBINED_FAST_STRUCT_LEGACY.size,
		):
			return

		payload_profile = {
			TM_COMBINED_FAST_STRUCT.size: "TM80",
			TM_COMBINED_FAST_STRUCT_WITH_ACCEL.size: "TM68",
			TM_COMBINED_FAST_STRUCT_WITH_QUAT.size: "TM62",
			TM_COMBINED_FAST_STRUCT_LEGACY.size: "TM54",
		}.get(len(payload), "TM?")

		if len(payload) == TM_COMBINED_FAST_STRUCT.size:
			(
				gyro_x_dps10,
				gyro_y_dps10,
				gyro_z_dps10,
				sensor_gyro_x_raw,
				sensor_gyro_y_raw,
				sensor_gyro_z_raw,
				body_gyro_x_raw,
				body_gyro_y_raw,
				body_gyro_z_raw,
				accel_x_raw,
				accel_y_raw,
				accel_z_raw,
				roll_cd,
				pitch_cd,
				yaw_cd,
				q0x10000,
				q1x10000,
				q2x10000,
				q3x10000,
				rc_throttle_us,
				rc_roll_us,
				rc_pitch_us,
				rc_yaw_us,
				rc_arm_us,
				motor1_us,
				motor2_us,
				motor3_us,
				motor4_us,
				_loop_dt_us,
				armed,
				failsafe_flags,
				telemetry_packet_counter,
				_command_packet_counter,
				_dropped_packet_counter,
				_crc_error_counter,
				_unknown_command_counter,
			) = TM_COMBINED_FAST_STRUCT.unpack(payload)
			fw_quaternion = (
				float(q0x10000) * 0.0001,
				float(q1x10000) * 0.0001,
				float(q2x10000) * 0.0001,
				float(q3x10000) * 0.0001,
			)
		elif len(payload) == TM_COMBINED_FAST_STRUCT_WITH_ACCEL.size:
			(
				gyro_x_dps10,
				gyro_y_dps10,
				gyro_z_dps10,
				accel_x_raw,
				accel_y_raw,
				accel_z_raw,
				roll_cd,
				pitch_cd,
				yaw_cd,
				q0x10000,
				q1x10000,
				q2x10000,
				q3x10000,
				rc_throttle_us,
				rc_roll_us,
				rc_pitch_us,
				rc_yaw_us,
				rc_arm_us,
				motor1_us,
				motor2_us,
				motor3_us,
				motor4_us,
				_loop_dt_us,
				armed,
				failsafe_flags,
				telemetry_packet_counter,
				_command_packet_counter,
				_dropped_packet_counter,
				_crc_error_counter,
				_unknown_command_counter,
			) = TM_COMBINED_FAST_STRUCT_WITH_ACCEL.unpack(payload)
			sensor_gyro_x_raw = int(gyro_x_dps10)
			sensor_gyro_y_raw = int(gyro_y_dps10)
			sensor_gyro_z_raw = int(gyro_z_dps10)
			body_gyro_x_raw = int(gyro_x_dps10)
			body_gyro_y_raw = int(gyro_y_dps10)
			body_gyro_z_raw = int(gyro_z_dps10)
			fw_quaternion = (
				float(q0x10000) * 0.0001,
				float(q1x10000) * 0.0001,
				float(q2x10000) * 0.0001,
				float(q3x10000) * 0.0001,
			)
		elif len(payload) == TM_COMBINED_FAST_STRUCT_WITH_QUAT.size:
			(
				gyro_x_dps10,
				gyro_y_dps10,
				gyro_z_dps10,
				roll_cd,
				pitch_cd,
				yaw_cd,
				q0x10000,
				q1x10000,
				q2x10000,
				q3x10000,
				rc_throttle_us,
				rc_roll_us,
				rc_pitch_us,
				rc_yaw_us,
				rc_arm_us,
				motor1_us,
				motor2_us,
				motor3_us,
				motor4_us,
				_loop_dt_us,
				armed,
				failsafe_flags,
				telemetry_packet_counter,
				_command_packet_counter,
				_dropped_packet_counter,
				_crc_error_counter,
				_unknown_command_counter,
			) = TM_COMBINED_FAST_STRUCT_WITH_QUAT.unpack(payload)
			sensor_gyro_x_raw = int(gyro_x_dps10)
			sensor_gyro_y_raw = int(gyro_y_dps10)
			sensor_gyro_z_raw = int(gyro_z_dps10)
			body_gyro_x_raw = int(gyro_x_dps10)
			body_gyro_y_raw = int(gyro_y_dps10)
			body_gyro_z_raw = int(gyro_z_dps10)
			accel_x_raw = 0
			accel_y_raw = 0
			accel_z_raw = 0
			fw_quaternion = (
				float(q0x10000) * 0.0001,
				float(q1x10000) * 0.0001,
				float(q2x10000) * 0.0001,
				float(q3x10000) * 0.0001,
			)
		else:
			(
				gyro_x_dps10,
				gyro_y_dps10,
				gyro_z_dps10,
				roll_cd,
				pitch_cd,
				yaw_cd,
				rc_throttle_us,
				rc_roll_us,
				rc_pitch_us,
				rc_yaw_us,
				rc_arm_us,
				motor1_us,
				motor2_us,
				motor3_us,
				motor4_us,
				_loop_dt_us,
				armed,
				failsafe_flags,
				telemetry_packet_counter,
				_command_packet_counter,
				_dropped_packet_counter,
				_crc_error_counter,
				_unknown_command_counter,
			) = TM_COMBINED_FAST_STRUCT_LEGACY.unpack(payload)
			sensor_gyro_x_raw = int(gyro_x_dps10)
			sensor_gyro_y_raw = int(gyro_y_dps10)
			sensor_gyro_z_raw = int(gyro_z_dps10)
			body_gyro_x_raw = int(gyro_x_dps10)
			body_gyro_y_raw = int(gyro_y_dps10)
			body_gyro_z_raw = int(gyro_z_dps10)
			accel_x_raw = 0
			accel_y_raw = 0
			accel_z_raw = 0
			fw_quaternion = None

		channels = [1500] * 16
		channels[0] = int(rc_roll_us)
		channels[1] = int(rc_pitch_us)
		channels[2] = int(rc_throttle_us)
		channels[3] = int(rc_yaw_us)
		channels[4] = int(rc_arm_us)

		with self.lock:
			if fw_quaternion is not None:
				pitch_deg, roll_deg, yaw_deg = self._full_range_pry_from_quaternion(fw_quaternion)
			else:
				pitch_deg = float(pitch_cd) / 100.0
				roll_deg = float(roll_cd) / 100.0
				yaw_deg = float(yaw_cd) / 100.0

			self.state.sequence = int(sequence)
			self.state.tm_profile = payload_profile
			self.state.tm_payload_len = len(payload)
			self.state.pitch_deg = pitch_deg
			self.state.roll_deg = roll_deg
			self.state.yaw_deg = yaw_deg
			self.state.fw_quaternion = fw_quaternion
			self.state.gx = int(sensor_gyro_x_raw)
			self.state.gy = int(sensor_gyro_y_raw)
			self.state.gz = int(sensor_gyro_z_raw)
			self.state.gx_filtered = int(body_gyro_x_raw)
			self.state.gy_filtered = int(body_gyro_y_raw)
			self.state.gz_filtered = int(body_gyro_z_raw)
			self.state.ax = int(accel_x_raw)
			self.state.ay = int(accel_y_raw)
			self.state.az = int(accel_z_raw)
			self.state.display_filter_cutoff_hz = 5
			self.state.pressure_pa = 0
			self.state.altitude_cm = 0
			self.state.battery_mv = None
			self.state.battery_voltage_v = None
			self.state.battery_adc_raw = 0
			self.state.battery_adc_channel = 255
			self.state.battery_adc_ready = 0
			self.state.desired_roll_rate_dps = 0.0
			self.state.desired_pitch_rate_dps = 0.0
			self.state.desired_yaw_rate_dps = 0.0
			self.state.gyro_roll_rate_dps = float(gyro_x_dps10) * 0.1
			self.state.gyro_pitch_rate_dps = float(gyro_y_dps10) * 0.1
			self.state.gyro_yaw_rate_dps = float(gyro_z_dps10) * 0.1
			self.state.roll_rate_error_dps = 0.0
			self.state.pitch_rate_error_dps = 0.0
			self.state.yaw_rate_error_dps = 0.0
			self.state.roll_pid_output = 0.0
			self.state.pitch_pid_output = 0.0
			self.state.yaw_pid_output = 0.0
			self.state.rc_link_ok = bool((failsafe_flags & 0x01) == 0)
			self.state.rc_frames = int(telemetry_packet_counter)
			self.state.channels = channels
			self.state.motor_mode = 1
			self.state.motors_armed = int(armed)
			self.state.gui_test_active = 0
			self.state.gyro_filter_available = True
			self.state.motors_us = [int(motor1_us), int(motor2_us), int(motor3_us), int(motor4_us)]
			self.state.last_update_monotonic = time.monotonic()
			self.state.connected = True

	def _candidate_ports(self) -> list[str]:
		if self.tcp_endpoint is not None:
			host, tcp_port = self.tcp_endpoint
			return [f"{host}:{tcp_port}"]

		ports: list[str] = []
		seen: set[str] = set()

		if self.port.lower() != "auto":
			ports.append(self.port)
			seen.add(self.port.upper())

		available = list(list_ports.comports())

		ranked: list[tuple[int, str]] = []
		for p in available:
			desc = p.description or ""
			if "STLink" in desc or "ST-LINK" in desc:
				continue
			key = p.device.upper()
			if key in seen:
				continue
			ranked.append((self._score_serial_port(p.device, desc, self._last_good_serial_port), p.device))

		ranked.sort(key=lambda item: item[0], reverse=True)
		for _, dev in ranked:
			ports.append(dev)
			seen.add(dev.upper())
		return ports

	def run(self) -> None:
		while self.running:
			if self.tcp_endpoint is not None:
				host, tcp_port = self.tcp_endpoint
				candidate = f"{host}:{tcp_port}"
				try:
					with socket.create_connection((host, tcp_port), timeout=2.0) as sock:
						sock.settimeout(SOCKET_READ_TIMEOUT_S)
						tm_parser = tm_protocol.StreamParser() if tm_protocol is not None else None
						with self.lock:
							self.state.connected = True
							self.state.port_name = candidate
							self.state.status_text = f"Connected: {candidate}"
						self._tcp_stream_state = "connected"

						last_packet_time = time.monotonic()
						while self.running:
							self._drain_tx_socket(sock)
							try:
								chunk = sock.recv(4096)
							except TimeoutError:
								chunk = b""
							except OSError:
								break

							if not chunk:
								idle_s = time.monotonic() - last_packet_time
								if (idle_s > 5.0) and (self._tcp_stream_state != "idle"):
									with self.lock:
										self.state.connected = True
										self.state.status_text = f"Connected: {candidate} (no TM frames {idle_s:0.1f}s)"
									self._tcp_stream_state = "idle"
								continue

							if tm_parser is None:
								with self.lock:
									self.state.connected = True
									self.state.tm_profile = "TM?"
									self.state.tm_payload_len = 0
									self.state.status_text = f"Connected: {candidate} (tm_protocol.py not available)"
								continue

							tm_frame_seen = False
							for frame in tm_parser.feed(chunk):
								if frame.packet_type == int(tm_protocol.PacketType.TM_COMBINED_FAST):
									self._apply_tm_combined_fast(candidate, frame.sequence, frame.payload)
									last_packet_time = time.monotonic()
									tm_frame_seen = True

							if tm_frame_seen and (self._tcp_stream_state != "tm"):
								with self.lock:
									self.state.connected = True
									self.state.status_text = f"Connected: {candidate}"
								self._tcp_stream_state = "tm"

							if not tm_frame_seen:
								idle_s = time.monotonic() - last_packet_time
								if (idle_s > 5.0) and (self._tcp_stream_state != "non-tm"):
									with self.lock:
										self.state.connected = True
										self.state.tm_profile = "NONE"
										self.state.tm_payload_len = 0
										self.state.status_text = f"Connected: {candidate} (non-TM stream {idle_s:0.1f}s)"
									self._tcp_stream_state = "non-tm"
				except OSError as exc:
					self.error_message = str(exc)
					with self.lock:
						self.state.connected = False
						self.state.status_text = f"Socket lost: {candidate} (retrying...)"
					time.sleep(0.3)
				continue

			candidates = self._candidate_ports()
			if not candidates:
				with self.lock:
					self.state.connected = False
					self.state.status_text = "Sniffing: no candidate serial ports"
				time.sleep(0.4)
				continue

			connected_once = False
			for candidate in candidates:
				if not self.running:
					break
				try:
					with serial.Serial(candidate, self.baud, timeout=SERIAL_READ_TIMEOUT_S) as ser:
						connected_once = True
						tm_parser = tm_protocol.StreamParser() if tm_protocol is not None else None
						with self.lock:
							self.state.connected = True
							self.state.port_name = candidate
							self.state.status_text = f"Connected: {candidate}"

						buffer = bytearray()
						last_packet_time = time.monotonic()
						valid_frame_seen = False
						while self.running:
							self._drain_tx_serial(ser)
							chunk = ser.read(ser.in_waiting or 1)
							if not chunk:
								idle_s = time.monotonic() - last_packet_time
								if idle_s > 2.0:
									with self.lock:
										self.state.connected = True
										self.state.status_text = f"Connected: {candidate} (telemetry idle {idle_s:0.1f}s)"
								if (self.port.lower() == "auto") and (not valid_frame_seen) and (idle_s > SERIAL_NO_FRAME_ROTATE_S):
									with self.lock:
										self.state.connected = False
										self.state.status_text = f"Skipping {candidate}: no telemetry frames"
									break
								continue

							tm_frame_seen = False
							if tm_parser is not None:
								for frame in tm_parser.feed(chunk):
									if frame.packet_type == int(tm_protocol.PacketType.TM_COMBINED_FAST):
										self._apply_tm_combined_fast(candidate, frame.sequence, frame.payload)
										last_packet_time = time.monotonic()
										tm_frame_seen = True
										valid_frame_seen = True
										self._last_good_serial_port = candidate

							legacy_frame_seen = False
							buffer.extend(chunk)
							for values in _decode_frames(buffer):
								self._apply_decoded(candidate, values)
								last_packet_time = time.monotonic()
								legacy_frame_seen = True
								valid_frame_seen = True
								self._last_good_serial_port = candidate

							if tm_frame_seen and (not legacy_frame_seen):
								with self.lock:
									self.state.connected = True
									self.state.status_text = f"Connected: {candidate}"
				except serial.SerialException as exc:
					self.error_message = str(exc)
					with self.lock:
						self.state.connected = False
						self.state.status_text = f"Port lost: {candidate} (sniffing...)"
					continue
				except Exception as exc:  # pragma: no cover
					self.error_message = str(exc)
					with self.lock:
						self.state.connected = False
						self.state.status_text = f"Reader error: {candidate} ({exc})"
					continue

			if not connected_once:
				with self.lock:
					self.state.connected = False
					self.state.status_text = "Sniffing serial ports..."
				time.sleep(0.3)


class TelemetryApp:
	def __init__(self, root: tk.Tk, state: TelemetryState, lock: threading.Lock, reader: SerialReader):
		self.root = root
		self.state = state
		self.lock = lock
		self.reader = reader

		root.title("KAKUTEH7 Telemetry GUI")
		root.geometry("1080x760")
		root.minsize(900, 620)

		self._layout_path = os.path.join(os.path.dirname(__file__), ".read_serial_gui_layout.json")
		self._saved_layout: dict[str, object] = self._load_layout_config()
		self._pane_order: list[tuple[str, ttk.Panedwindow]] = []

		self.header_var = tk.StringVar(value="Waiting for data...")
		self.imu_var = tk.StringVar(value="")
		self.baro_var = tk.StringVar(value="")
		self.battery_var = tk.StringVar(value="BAT n/a")
		self.filter_target_var = tk.StringVar(value="Requested cutoff: 5 Hz")
		self.filter_fw_var = tk.StringVar(value="Firmware cutoff: 5 Hz")
		self.performance_mode_var = tk.StringVar(value=PERF_MODE_NORMAL)
		self._filter_command_after_id: str | None = None
		self._last_filter_sent = 5
		self._ui_interval_ms = GUI_UPDATE_INTERVAL_MS

		header = ttk.Frame(root, padding=10)
		header.pack(fill=tk.X)
		ttk.Label(header, textvariable=self.imu_var, font=("Consolas", 11)).pack(anchor=tk.W)

		perf_row = ttk.Frame(header)
		perf_row.pack(fill=tk.X, pady=(4, 0))
		ttk.Label(perf_row, text="Performance:").pack(side=tk.LEFT)
		perf_combo = ttk.Combobox(
			perf_row,
			width=14,
			textvariable=self.performance_mode_var,
			values=(PERF_MODE_NORMAL, PERF_MODE_LOW_LATENCY),
			state="readonly",
		)
		perf_combo.pack(side=tk.LEFT, padx=(8, 0))
		perf_combo.bind("<<ComboboxSelected>>", self._on_performance_mode_changed)

		body = ttk.Frame(root, padding=(10, 0, 10, 10))
		body.pack(fill=tk.BOTH, expand=True)

		root_stack = ttk.Panedwindow(body, orient=tk.VERTICAL)
		root_stack.pack(fill=tk.BOTH, expand=True)
		self._pane_order.append(("root_stack", root_stack))

		top_row = ttk.Panedwindow(root_stack, orient=tk.HORIZONTAL)
		root_stack.add(top_row, weight=3)
		self._pane_order.append(("top_row", top_row))

		board3d_card = ttk.Frame(top_row)
		ttk.Label(board3d_card, text="Kakute 3D Body", font=("Segoe UI", 12, "bold")).pack(anchor=tk.W, pady=(0, 6))
		self.board3d_canvas = tk.Canvas(board3d_card, width=360, height=260, bg="#ffffff", highlightthickness=0)
		self.board3d_canvas.pack(fill=tk.BOTH, expand=True)
		top_row.add(board3d_card, weight=2)

		channel_card = ttk.Frame(top_row)
		ttk.Label(channel_card, text="Channels", font=("Segoe UI", 12, "bold")).pack(anchor=tk.W, pady=(0, 6))
		self.channel_canvas = tk.Canvas(channel_card, bg="#0f1319", highlightthickness=0)
		self.channel_canvas.pack(fill=tk.BOTH, expand=True)
		top_row.add(channel_card, weight=2)

		filter_card = ttk.Frame(top_row)
		ttk.Label(filter_card, text="Sensor Graphs", font=("Segoe UI", 12, "bold")).pack(anchor=tk.W, pady=(0, 6))

		filter_row = ttk.Frame(filter_card)
		filter_row.pack(fill=tk.X, padx=2, pady=(0, 6))
		ttk.Label(filter_row, textvariable=self.filter_target_var, font=("Consolas", 10, "bold")).pack(side=tk.LEFT)
		ttk.Label(filter_row, textvariable=self.filter_fw_var, font=("Consolas", 10), foreground="#94a3b8").pack(side=tk.RIGHT)

		self.filter_scale = ttk.Scale(filter_card, from_=0, to=10, orient=tk.HORIZONTAL, command=self._on_filter_cutoff_changed)
		self.filter_scale.set(5)
		self.filter_scale.pack(fill=tk.X, padx=2, pady=(0, 6))

		self.gyro_canvas = tk.Canvas(filter_card, height=260, bg="#0f1720", highlightthickness=0)
		self.gyro_canvas.pack(fill=tk.BOTH, expand=True)
		top_row.add(filter_card, weight=3)

		bottom_row = ttk.Panedwindow(root_stack, orient=tk.HORIZONTAL)
		root_stack.add(bottom_row, weight=2)
		self._pane_order.append(("bottom_row", bottom_row))

		stick_card = ttk.Frame(bottom_row)
		ttk.Label(stick_card, text="RC Sticks", font=("Segoe UI", 12, "bold")).pack(anchor=tk.W, pady=(0, 6))
		self.stick_canvas = tk.Canvas(stick_card, width=560, height=280, bg="#151a22", highlightthickness=0)
		self.stick_canvas.pack(fill=tk.BOTH, expand=True)
		bottom_row.add(stick_card, weight=2)

		controls_and_motors = ttk.Panedwindow(bottom_row, orient=tk.VERTICAL)
		bottom_row.add(controls_and_motors, weight=2)
		self._pane_order.append(("controls_and_motors", controls_and_motors))

		control_card = ttk.LabelFrame(controls_and_motors, text="Single Motor Test (USB)")
		self.test_motor_var = tk.IntVar(value=1)
		self.test_us_var = tk.IntVar(value=1100)
		self.test_running = False
		self.test_after_id: str | None = None

		row1 = ttk.Frame(control_card)
		row1.pack(fill=tk.X, padx=8, pady=(8, 4))
		ttk.Label(row1, text="Motor").pack(side=tk.LEFT)
		ttk.Combobox(row1, width=6, textvariable=self.test_motor_var, values=(1, 2, 3, 4), state="readonly").pack(side=tk.LEFT, padx=(8, 14))
		ttk.Label(row1, text="Power (us)").pack(side=tk.LEFT)
		self.test_us_scale = ttk.Scale(row1, from_=1000, to=2000, orient=tk.HORIZONTAL, command=self._on_test_us_changed)
		self.test_us_scale.set(1100)
		self.test_us_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(8, 8))
		self.test_us_label = ttk.Label(row1, width=8, text="1100 us")
		self.test_us_label.pack(side=tk.LEFT)

		row2 = ttk.Frame(control_card)
		row2.pack(fill=tk.X, padx=8, pady=(2, 8))
		self.test_button = ttk.Button(row2, text="Hold To Run")
		self.test_button.pack(side=tk.LEFT)
		self.test_button.bind("<ButtonPress-1>", self._on_test_press)
		self.test_button.bind("<ButtonRelease-1>", self._on_test_release)
		self.test_button.bind("<Leave>", self._on_test_release)
		ttk.Label(row2, text="Sends command every 80 ms with 250 ms firmware timeout.").pack(side=tk.LEFT, padx=(10, 0))
		controls_and_motors.add(control_card, weight=1)

		motor_card = ttk.Frame(controls_and_motors)
		ttk.Label(motor_card, text="Motors (M1-M4)", font=("Segoe UI", 12, "bold")).pack(anchor=tk.W, pady=(0, 6))
		self.motor_canvas = tk.Canvas(motor_card, width=520, height=220, bg="#10161d", highlightthickness=0)
		self.motor_canvas.pack(fill=tk.BOTH, expand=True)
		controls_and_motors.add(motor_card, weight=2)

		self.footer_var = tk.StringVar(value="")
		ttk.Label(root, textvariable=self.footer_var, font=("Consolas", 10), padding=(10, 0, 10, 8)).pack(anchor=tk.W)
		self._gyro_raw_history = deque(maxlen=240)
		self._gyro_filtered_history = deque(maxlen=240)
		self._gyro_time_history = deque(maxlen=240)

		self._apply_performance_mode(self.performance_mode_var.get())
		self._apply_saved_layout_async()

	def _apply_performance_mode(self, mode: str) -> None:
		self._ui_interval_ms = 33 if mode == PERF_MODE_NORMAL else GUI_UPDATE_INTERVAL_MS

	def _on_performance_mode_changed(self, _event=None) -> None:
		self._apply_performance_mode(self.performance_mode_var.get())

	def _load_layout_config(self) -> dict[str, object]:
		try:
			with open(self._layout_path, "r", encoding="utf-8") as handle:
				loaded = json.load(handle)
			if isinstance(loaded, dict) and int(loaded.get("schema", 0)) == LAYOUT_SCHEMA_VERSION:
				return loaded
		except Exception:
			pass
		return {}

	def _save_layout_config(self) -> None:
		panes: dict[str, list[int]] = {}
		for name, pane in self._pane_order:
			try:
				pane_count = len(pane.panes())
				if pane_count < 2:
					continue
				panes[name] = [int(pane.sashpos(i)) for i in range(pane_count - 1)]
			except Exception:
				continue
		payload = {"schema": LAYOUT_SCHEMA_VERSION, "geometry": self.root.winfo_geometry(), "panes": panes}
		try:
			with open(self._layout_path, "w", encoding="utf-8") as handle:
				json.dump(payload, handle, indent=2)
		except Exception:
			pass

	def _apply_saved_layout_async(self) -> None:
		geometry = self._saved_layout.get("geometry")
		if isinstance(geometry, str) and geometry:
			try:
				self.root.geometry(geometry)
			except Exception:
				pass
		self.root.after(120, self._apply_saved_panes)

	def _apply_saved_panes(self) -> None:
		self.root.update_idletasks()
		pane_data = self._saved_layout.get("panes")
		if not isinstance(pane_data, dict):
			return
		for name, pane in self._pane_order:
			saved_positions = pane_data.get(name)
			if not isinstance(saved_positions, list):
				continue
			pane_orient = str(pane.cget("orient"))
			pane_span = pane.winfo_width() if pane_orient == "horizontal" else pane.winfo_height()
			if pane_span <= 0:
				continue
			edge_margin = 120
			for idx, value in enumerate(saved_positions):
				try:
					requested = int(value)
					clamped = max(edge_margin, min(pane_span - edge_margin, requested))
					pane.sashpos(idx, clamped)
				except Exception:
					continue

	def on_close(self) -> None:
		self._save_layout_config()

	def _queue_motor_test(self, motor_index: int, motor_us: int) -> None:
		self.reader.queue_command(_build_motor_test_packet(motor_index, motor_us))
		self.reader.queue_command(_build_motor_test_ascii(motor_index, motor_us))

	def _queue_display_filter(self, cutoff_hz: int) -> None:
		self.reader.queue_command(_build_display_filter_packet(cutoff_hz))
		self.reader.queue_command(_build_display_filter_ascii(cutoff_hz))

	def _send_display_filter_now(self, cutoff_hz: int) -> None:
		cutoff = max(0, min(10, int(cutoff_hz)))
		self._last_filter_sent = cutoff
		self.filter_target_var.set(f"Requested cutoff: {cutoff} Hz")
		self._queue_display_filter(cutoff)

	def _on_filter_cutoff_changed(self, value: str) -> None:
		cutoff = int(float(value))
		self.filter_target_var.set(f"Requested cutoff: {cutoff} Hz")
		if self._filter_command_after_id is not None:
			self.root.after_cancel(self._filter_command_after_id)
		self._filter_command_after_id = self.root.after(35, lambda cutoff=cutoff: self._send_display_filter_now(cutoff))

	def _on_test_us_changed(self, value: str) -> None:
		us = int(float(value))
		self.test_us_var.set(us)
		if hasattr(self, "test_us_label"):
			self.test_us_label.configure(text=f"{us:4d} us")
		if self.test_running:
			self._queue_motor_test(self.test_motor_var.get(), us)

	def _test_tick(self) -> None:
		if not self.test_running:
			self.test_after_id = None
			return
		self._queue_motor_test(self.test_motor_var.get(), self.test_us_var.get())
		self.test_after_id = self.root.after(80, self._test_tick)

	def _on_test_press(self, _event=None) -> None:
		if self.test_running:
			return
		self.test_running = True
		self._queue_motor_test(self.test_motor_var.get(), self.test_us_var.get())
		self._test_tick()

	def _on_test_release(self, _event=None) -> None:
		if not self.test_running:
			return
		self.test_running = False
		if self.test_after_id is not None:
			self.root.after_cancel(self.test_after_id)
			self.test_after_id = None
		self._queue_motor_test(self.test_motor_var.get(), 1000)

	def _channel_to_norm(self, raw: int) -> float:
		clamped = max(999, min(2000, raw))
		return ((clamped - 999) / 1001.0) * 2.0 - 1.0

	def _draw_sticks(self, ch: list[int]) -> None:
		c = self.stick_canvas
		c.delete("all")
		w = c.winfo_width()
		h = c.winfo_height()
		pad = 30
		size = min((w - pad * 3) / 2, h - 60)
		y0 = (h - size) / 2
		left_x0 = pad
		right_x0 = pad * 2 + size
		self._draw_single_stick(c, left_x0, y0, size, self._channel_to_norm(ch[3]), self._channel_to_norm(ch[2]), "Left CH4/CH3")
		self._draw_single_stick(c, right_x0, y0, size, self._channel_to_norm(ch[0]), self._channel_to_norm(ch[1]), "Right CH1/CH2")

	def _draw_single_stick(self, c: tk.Canvas, x0: float, y0: float, size: float, x_norm: float, y_norm: float, label: str) -> None:
		x1 = x0 + size
		y1 = y0 + size
		c.create_rectangle(x0, y0, x1, y1, outline="#9ca3af", width=2)
		c.create_line((x0 + x1) / 2, y0, (x0 + x1) / 2, y1, fill="#4b5563")
		c.create_line(x0, (y0 + y1) / 2, x1, (y0 + y1) / 2, fill="#4b5563")
		px = x0 + ((x_norm + 1.0) * 0.5) * size
		py = y0 + (1.0 - (y_norm + 1.0) * 0.5) * size
		c.create_oval(px - 8, py - 8, px + 8, py + 8, fill="#22d3ee", outline="")
		c.create_text(x0, y0 - 8, anchor="sw", fill="#e5e7eb", font=("Consolas", 11), text=f"{label}  X {x_norm:+.2f} Y {y_norm:+.2f}")

	def _draw_channels(self, ch: list[int]) -> None:
		c = self.channel_canvas
		c.delete("all")
		w = c.winfo_width()
		left = 75
		right = w - 24
		top = 24
		row_h = 22
		for i, raw in enumerate(ch, start=1):
			y = top + (i - 1) * row_h
			n = self._channel_to_norm(raw)
			zero_x = left + (right - left) * 0.5
			val_x = left + (right - left) * ((n + 1.0) * 0.5)
			c.create_text(10, y + 8, anchor="w", fill="#e2e8f0", font=("Consolas", 10), text=f"CH{i:02d}")
			c.create_rectangle(left, y, right, y + 14, outline="#374151", width=1)
			c.create_line(zero_x, y, zero_x, y + 14, fill="#4b5563")
			if val_x >= zero_x:
				c.create_rectangle(zero_x, y + 1, val_x, y + 13, fill="#16a34a", outline="")
			else:
				c.create_rectangle(val_x, y + 1, zero_x, y + 13, fill="#2563eb", outline="")
			c.create_text(right + 6, y + 8, anchor="w", fill="#cbd5e1", font=("Consolas", 10), text=f"{raw:4d}")

	def _draw_motors(self, mode: int, armed: int, motors_us: list[int]) -> None:
		c = self.motor_canvas
		c.delete("all")
		w = c.winfo_width()
		h = c.winfo_height()
		cx = w / 2.0
		cy = h / 2.0 + 12.0
		dx = 120.0
		dy = 65.0
		r = 30.0
		mode_text = "REAL THROTTLE" if mode == 1 else "SIMULATION"
		mode_color = "#ef4444" if mode == 1 else "#f59e0b"
		arm_text = "ARMED" if armed else "DISARMED"
		arm_color = "#22c55e" if armed else "#94a3b8"
		c.create_text(10, 10, anchor="nw", fill="#e5e7eb", font=("Consolas", 11, "bold"), text=f"MODE: {mode_text}")
		c.create_text(220, 10, anchor="nw", fill=arm_color, font=("Consolas", 11, "bold"), text=f"STATE: {arm_text}")
		c.create_text(cx, 14, anchor="n", fill="#fbbf24", font=("Consolas", 11, "bold"), text="FRONT")
		c.create_line(cx, 30, cx, 52, fill="#fbbf24", width=3, arrow=tk.FIRST)
		c.create_line(cx - 70, cy - 38, cx + 70, cy + 38, fill="#334155", width=2)
		c.create_line(cx + 70, cy - 38, cx - 70, cy + 38, fill="#334155", width=2)
		positions = [(cx + dx, cy + dy), (cx + dx, cy - dy), (cx - dx, cy + dy), (cx - dx, cy - dy)]
		role_labels = ["AR", "FR", "AL", "FL"]
		for idx, (mx, my) in enumerate(positions):
			us = int(motors_us[idx]) if idx < len(motors_us) else 1000
			norm = max(0.0, min(1.0, (us - 1000) / 1000.0))
			fill = "#1f2937" if us <= 1000 else mode_color
			c.create_oval(mx - r, my - r, mx + r, my + r, fill=fill, outline="#64748b", width=2)
			c.create_arc(mx - r, my - r, mx + r, my + r, start=90, extent=-360.0 * norm, style=tk.ARC, outline="#22c55e", width=4)
			c.create_text(mx, my - 8, fill="#e2e8f0", font=("Consolas", 10, "bold"), text=f"M{idx + 1}")
			c.create_text(mx, my + 4, fill="#93c5fd", font=("Consolas", 9, "bold"), text=role_labels[idx])
			c.create_text(mx, my + 16, fill="#cbd5e1", font=("Consolas", 9), text=f"{us:4d} us")

	def _append_gyro_history(self, snapshot: TelemetryState) -> None:
		raw_scale = IMU_GYRO_DPS_PER_LSB
		raw_sample = (snapshot.gx * raw_scale, snapshot.gy * raw_scale, snapshot.gz * raw_scale)
		filtered_sample = (
			snapshot.gx_filtered * raw_scale,
			snapshot.gy_filtered * raw_scale,
			snapshot.gz_filtered * raw_scale,
		) if snapshot.gyro_filter_available else raw_sample
		self._gyro_raw_history.append(raw_sample)
		self._gyro_filtered_history.append(filtered_sample)
		self._gyro_time_history.append(float(snapshot.last_update_monotonic) if snapshot.last_update_monotonic > 0.0 else time.monotonic())

	def _draw_gyro_graph(self, snapshot: TelemetryState) -> None:
		c = self.gyro_canvas
		c.delete("all")
		w = c.winfo_width()
		h = c.winfo_height()
		split = h // 2
		c.create_rectangle(0, 0, w, h, fill="#0f1720", outline="")
		c.create_text(10, 8, anchor="nw", fill="#e5e7eb", font=("Consolas", 11, "bold"), text=f"Gyro Display Filter  {snapshot.display_filter_cutoff_hz} Hz")

		def draw_plot(y0: int, y1: int, history: deque[tuple[float, float, float]], title: str) -> None:
			pad_x = 18
			top = y0 + 22
			bottom = y1 - 14
			left = pad_x
			right = w - pad_x
			center_y = (top + bottom) / 2.0
			max_abs_dps = max((abs(v) for sample in history for v in sample), default=0.0)
			amp_dps = max(20.0, min(2000.0, max_abs_dps * 1.2))
			pixels_per_dps = ((bottom - top) * 0.5) / amp_dps
			c.create_text(left, y0 + 4, anchor="nw", fill="#cbd5e1", font=("Consolas", 10, "bold"), text=title)
			c.create_line(left, center_y, right, center_y, fill="#475569")
			c.create_line(left, top, right, top, fill="#1f2937")
			c.create_line(left, bottom, right, bottom, fill="#1f2937")
			c.create_text(left, top + 2, anchor="nw", fill="#94a3b8", font=("Consolas", 9), text=f"+/-{amp_dps:5.1f} dps")
			if len(history) < 2:
				return
			denom = max(1, len(history) - 1)
			axis_colors = ("#f87171", "#4ade80", "#60a5fa")
			for axis in range(3):
				points: list[float] = []
				for idx, sample in enumerate(history):
					x_pos = left + (right - left) * (idx / denom)
					clamped = max(-amp_dps, min(amp_dps, sample[axis]))
					y_pos = center_y - clamped * pixels_per_dps
					points.extend((x_pos, y_pos))
				c.create_line(*points, fill=axis_colors[axis], width=2, smooth=True)

		draw_plot(0, split, self._gyro_raw_history, "Raw Gyro (dps)")
		draw_plot(split, h, self._gyro_filtered_history, "Filtered Gyro (dps)")

	@staticmethod
	def _full_range_pry_from_quaternion(quaternion: tuple[float, float, float, float]) -> tuple[float, float, float]:
		qw, qx, qy, qz = (float(v) for v in quaternion)
		sin_pitch = 2.0 * ((qw * qy) - (qz * qx))
		sin_pitch = max(-1.0, min(1.0, sin_pitch))
		roll_deg = math.degrees(math.atan2(2.0 * ((qw * qx) + (qy * qz)), 1.0 - (2.0 * ((qx * qx) + (qy * qy)))))
		pitch_deg = math.degrees(math.asin(sin_pitch))
		yaw_deg = math.degrees(math.atan2(2.0 * ((qw * qz) + (qx * qy)), 1.0 - (2.0 * ((qy * qy) + (qz * qz)))))
		if yaw_deg > 180.0:
			yaw_deg -= 360.0
		elif yaw_deg < -180.0:
			yaw_deg += 360.0
		return pitch_deg, roll_deg, yaw_deg

	def _draw_board_3d(self, pitch: float, roll: float, yaw: float, quaternion: tuple[float, float, float, float] | None) -> None:
		c = self.board3d_canvas
		c.delete("all")
		w = c.winfo_width()
		h = c.winfo_height()
		cx = w / 2.0
		cy = h / 2.0

		bh = 9.0
		z_top = -bh
		z_bottom = bh

		# Asymmetric hull geometry makes full 180-degree swings obvious.
		nose = (110.0, 0.0, z_top)
		right_front = (15.0, 62.0, z_top)
		right_rear = (-62.0, 46.0, z_top)
		left_rear = (-70.0, -58.0, z_top)
		left_front = (22.0, -78.0, z_top)
		tail = (-100.0, 0.0, z_top)
		hull = [nose, right_front, right_rear, left_rear, left_front]
		hull_bottom = [(x, y, z_bottom) for (x, y, _) in hull]

		# Offset fin on the right side to avoid mirror ambiguity in roll/pitch.
		fin_base_a = (-78.0, 24.0, z_top)
		fin_base_b = (-96.0, 24.0, z_top)
		fin_tip = (-87.0, 24.0, z_top - 38.0)
		axis_len = 120.0
		axis_origin = (0.0, 0.0, 0.0)
		axis_x_fwd = (axis_len, 0.0, 0.0)
		axis_y_right = (0.0, axis_len, 0.0)
		axis_z_down = (0.0, 0.0, axis_len)

		# Render geometry from quaternion orientation when available.
		# Full-range angles are kept for display text, but are not Euler angles.
		if quaternion is not None:
			qw, qx, qy, qz = (float(v) for v in quaternion)
			sinp = (2.0 * ((qw * qy) - (qz * qx)))
			sinp = max(-1.0, min(1.0, sinp))
			pr = math.asin(sinp)
			rr = math.atan2(2.0 * ((qw * qx) + (qy * qz)), 1.0 - (2.0 * ((qx * qx) + (qy * qy))))
			yr = math.atan2(2.0 * ((qw * qz) + (qx * qy)), 1.0 - (2.0 * ((qy * qy) + (qz * qz))))
		else:
			pr = math.radians(pitch)
			rr = math.radians(roll)
			yr = math.radians(yaw)

		cp = math.cos(pr)
		sp = math.sin(pr)
		cr = math.cos(rr)
		sr = math.sin(rr)
		cyaw = math.cos(yr)
		syaw = math.sin(yr)
		r00 = cyaw * cp
		r01 = cyaw * sp * sr - syaw * cr
		r02 = cyaw * sp * cr + syaw * sr
		r10 = syaw * cp
		r11 = syaw * sp * sr + cyaw * cr
		r12 = syaw * sp * cr - cyaw * sr
		r20 = -sp
		r21 = cp * sr
		r22 = cp * cr

		def rotate(v: tuple[float, float, float]) -> tuple[float, float, float]:
			x, y, z = v
			return (
				r00 * x + r01 * y + r02 * z,
				r10 * x + r11 * y + r12 * z,
				r20 * x + r21 * y + r22 * z,
			)

		def project(v: tuple[float, float, float]) -> tuple[float, float]:
			x, y, z = v
			x_view = (1.00 * y) + (0.10 * x)
			y_view = (-1.00 * z) + (0.05 * x)
			z_view = -1.00 * x
			dist = 520.0
			scale = dist / (dist - z_view)
			return (cx + x_view * scale * 1.2, cy - y_view * scale * 1.2)

		hull_pts = [project(rotate(v)) for v in hull]
		hull_bottom_pts = [project(rotate(v)) for v in hull_bottom]
		nose_pt = project(rotate(nose))
		tail_pt = project(rotate(tail))
		fin_pts = [project(rotate(v)) for v in (fin_base_a, fin_base_b, fin_tip)]

		hashmarks_local = [
			((72.0, -8.0, z_top), (72.0, 8.0, z_top)),
			((44.0, -10.0, z_top), (44.0, 10.0, z_top)),
			((16.0, -12.0, z_top), (16.0, 12.0, z_top)),
			((-12.0, -12.0, z_top), (-12.0, 12.0, z_top)),
			((-40.0, -10.0, z_top), (-40.0, 10.0, z_top)),
		]
		hashmark_pts = [(project(rotate(a)), project(rotate(b))) for (a, b) in hashmarks_local]
		axis_o = project(rotate(axis_origin))
		axis_x = project(rotate(axis_x_fwd))
		axis_y = project(rotate(axis_y_right))
		axis_z = project(rotate(axis_z_down))
		c.create_rectangle(0, 0, w, h, fill="#ffffff", outline="")
		c.create_polygon(*[coord for p in hull_bottom_pts for coord in p], fill="#0f172a", outline="#334155", width=1)
		for i in range(len(hull_pts)):
			next_i = (i + 1) % len(hull_pts)
			side_quad = [
				hull_pts[i],
				hull_pts[next_i],
				hull_bottom_pts[next_i],
				hull_bottom_pts[i],
			]
			c.create_polygon(*[coord for p in side_quad for coord in p], fill="#172554", outline="#1e293b", width=1)

		c.create_polygon(*[coord for p in hull_pts for coord in p], fill="#1e3a8a", stipple="gray25", outline="#93c5fd", width=2)
		c.create_polygon(*[coord for p in fin_pts for coord in p], fill="#334155", outline="#cbd5e1", width=2)
		c.create_line(nose_pt[0], nose_pt[1], tail_pt[0], tail_pt[1], fill="#fbbf24", width=3)
		for a, b in hashmark_pts:
			c.create_line(a[0], a[1], b[0], b[1], fill="#f8fafc", width=2)
		c.create_oval(
			hull_pts[1][0] - 4,
			hull_pts[1][1] - 4,
			hull_pts[1][0] + 4,
			hull_pts[1][1] + 4,
			fill="#22d3ee",
			outline="",
		)

		# Exact body reference frame (FRD): +X forward, +Y right, +Z down.
		c.create_line(axis_o[0], axis_o[1], axis_x[0], axis_x[1], fill="#ef4444", width=3, arrow=tk.LAST)
		c.create_line(axis_o[0], axis_o[1], axis_y[0], axis_y[1], fill="#22c55e", width=3, arrow=tk.LAST)
		c.create_line(axis_o[0], axis_o[1], axis_z[0], axis_z[1], fill="#3b82f6", width=3, arrow=tk.LAST)
		c.create_text(axis_x[0] + 8, axis_x[1], anchor="w", fill="#000000", font=("Consolas", 10, "bold"), text="+X FORWARD")
		c.create_text(axis_y[0] + 8, axis_y[1], anchor="w", fill="#000000", font=("Consolas", 10, "bold"), text="+Y RIGHT")
		c.create_text(axis_z[0] + 8, axis_z[1], anchor="w", fill="#000000", font=("Consolas", 10, "bold"), text="+Z DOWN")

		pose_source = "Telemetry Full-Range"
		c.create_text(10, 10, anchor="nw", fill="#000000", font=("Consolas", 10, "bold"), text=f"3D Pose ({pose_source})  Body Frame: FRD")
		c.create_text(10, 28, anchor="nw", fill="#000000", font=("Consolas", 9), text="Asymmetric hull: yellow nose-line + right fin")
		if quaternion is not None:
			c.create_text(10, 44, anchor="nw", fill="#000000", font=("Consolas", 9), text="Model rotation source: firmware quaternion")
		c.create_text(10, h - 12, anchor="sw", fill="#000000", font=("Consolas", 10), text=f"P {pitch:+6.2f}  R {roll:+6.2f}  Yrel {yaw:+6.2f}")

	def update(self) -> None:
		with self.lock:
			snapshot = TelemetryState(**self.state.__dict__)

		age_ms = (time.monotonic() - snapshot.last_update_monotonic) * 1000.0 if snapshot.last_update_monotonic else -1.0
		link = "OK" if snapshot.rc_link_ok else "--"
		ser = "UP" if snapshot.connected else "DN"
		port = snapshot.port_name if snapshot.port_name else "n/a"
		motor_mode_text = "REAL" if snapshot.motor_mode == 1 else "SIM"
		motor_state_text = "ARM" if snapshot.motors_armed else "DIS"
		gui_test_text = "GUI_TEST" if snapshot.gui_test_active else "-"
		board_quaternion = snapshot.fw_quaternion
		board_angle_source = "Telemetry Full-Range"
		board_pitch_deg = snapshot.pitch_deg
		board_roll_deg = snapshot.roll_deg
		board_yaw_deg = snapshot.yaw_deg
		self.header_var.set(
			f"SEQ {snapshot.sequence:06d}   RC {link}   RF {snapshot.rc_frames:6d}   "
			f"MOT {motor_mode_text}/{motor_state_text}/{gui_test_text}   SER {ser} ({port})   Age {age_ms:6.1f} ms"
		)
		self.imu_var.set(
			f"Sensor Gyro[Gx={snapshot.gx * IMU_GYRO_DPS_PER_LSB:>7.2f}, Gy={snapshot.gy * IMU_GYRO_DPS_PER_LSB:>7.2f}, Gz={snapshot.gz * IMU_GYRO_DPS_PER_LSB:>7.2f}]   "
			f"Sensor Accel[Ax={snapshot.ax / IMU_ACCEL_LSB_PER_G:>7.3f}, Ay={snapshot.ay / IMU_ACCEL_LSB_PER_G:>7.3f}, Az={snapshot.az / IMU_ACCEL_LSB_PER_G:>7.3f}]\n"
			f"Body   Gyro[Bx={snapshot.gx_filtered * IMU_GYRO_DPS_PER_LSB:>7.2f}, By={snapshot.gy_filtered * IMU_GYRO_DPS_PER_LSB:>7.2f}, Bz={snapshot.gz_filtered * IMU_GYRO_DPS_PER_LSB:>7.2f}]   "
			f"Body   Accel[Bx={-snapshot.ay / IMU_ACCEL_LSB_PER_G:>7.3f}, By={-snapshot.ax / IMU_ACCEL_LSB_PER_G:>7.3f}, Bz={-snapshot.az / IMU_ACCEL_LSB_PER_G:>7.3f}]\n"
			f"3D Axis Angles[X(roll)={board_roll_deg:+7.2f} deg, Y(pitch)={board_pitch_deg:+7.2f} deg, Z(yaw rel)={board_yaw_deg:+7.2f} deg]   "
			f"3D Source[{board_angle_source}]"
		)
		self.baro_var.set(f"BARO P {snapshot.pressure_pa:7d} Pa   ALT {snapshot.altitude_cm / 100.0:+7.2f} m")
		if snapshot.battery_voltage_v is None:
			self.battery_var.set("BAT n/a")
		elif snapshot.battery_mv == 0:
			self.battery_var.set("BAT 0.00 V")
		else:
			self.battery_var.set(f"BAT {snapshot.battery_voltage_v:0.2f} V")
		self.filter_fw_var.set(f"Firmware cutoff: {snapshot.display_filter_cutoff_hz} Hz")
		self.footer_var.set(
			f"{GUI_BUILD_TAG} | {snapshot.status_text} | TM {snapshot.tm_profile} len {snapshot.tm_payload_len:3d} | "
			f"Motor us: M1 {snapshot.motors_us[0]:4d} M2 {snapshot.motors_us[1]:4d} "
			f"M3 {snapshot.motors_us[2]:4d} M4 {snapshot.motors_us[3]:4d} | Desired rate dps "
			f"R {snapshot.desired_roll_rate_dps:+6.1f} P {snapshot.desired_pitch_rate_dps:+6.1f} Y {snapshot.desired_yaw_rate_dps:+6.1f} "
			f"| Gyro dps R {snapshot.gyro_roll_rate_dps:+6.1f} P {snapshot.gyro_pitch_rate_dps:+6.1f} Y {snapshot.gyro_yaw_rate_dps:+6.1f} "
			f"| PID R {snapshot.roll_pid_output:+0.3f} P {snapshot.pitch_pid_output:+0.3f} Y {snapshot.yaw_pid_output:+0.3f}"
		)
		self._append_gyro_history(snapshot)
		self._draw_board_3d(board_pitch_deg, board_roll_deg, board_yaw_deg, board_quaternion)
		self._draw_sticks(snapshot.channels)
		self._draw_gyro_graph(snapshot)
		self._draw_channels(snapshot.channels)
		self._draw_motors(snapshot.motor_mode, snapshot.motors_armed, snapshot.motors_us)
		self.root.after(self._ui_interval_ms, self.update)


def main() -> int:
	parser = argparse.ArgumentParser(description="Graphical telemetry viewer for serial or TCP telemetry")
	parser.add_argument("--port", default="COM6", help="Serial COM port (e.g. COM6/auto) or TCP host:port")
	parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
	parser.add_argument("--verbose", action="store_true", help="Print serial diagnostics to console")
	args = parser.parse_args()

	if args.port.lower() == "auto":
		args.port = _pick_port()

	if not args.port:
		ports = list(_list_ports())
		if args.verbose:
			if not ports:
				print("No serial ports found.")
			else:
				print("Available serial ports:")
				for p in ports:
					print(f"  {p}")
		return 1

	state = TelemetryState()
	lock = threading.Lock()
	reader = SerialReader(args.port, args.baud, state, lock)
	reader.start()

	root = tk.Tk()
	app = TelemetryApp(root, state, lock, reader)

	def _on_close() -> None:
		app.on_close()
		reader.stop()
		root.destroy()

	root.protocol("WM_DELETE_WINDOW", _on_close)
	app.update()
	root.mainloop()

	if reader.error_message:
		if args.verbose:
			print(f"Serial error on {args.port}: {reader.error_message}")
		return 1
	return 0


if __name__ == "__main__":
	raise SystemExit(main())
