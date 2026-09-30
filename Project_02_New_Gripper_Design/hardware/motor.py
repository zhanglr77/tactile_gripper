"""DM-H3510 direct-drive motor driver — MIT mode CAN protocol.

Supports four CAN backends: Linux SocketCAN, python-can (Windows),
DAMIAO USB2CAN serial, and Waveshare USB-CAN-A serial.
"""
from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
import socket
import struct
import time
from typing import Optional


CAN_FRAME = struct.Struct("=IB3x8s")
ENABLE = b"\xff\xff\xff\xff\xff\xff\xff\xfc"
DISABLE = b"\xff\xff\xff\xff\xff\xff\xff\xfd"
ERRORS = {
    0x0: "disabled", 0x1: "enabled", 0x8: "over-voltage",
    0x9: "under-voltage", 0xA: "over-current", 0xB: "MOS over-temperature",
    0xC: "motor over-temperature", 0xD: "communication lost", 0xE: "overload",
}


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def float_to_uint(value: float, low: float, high: float, bits: int) -> int:
    value = clamp(value, low, high)
    return int(round((value - low) * ((1 << bits) - 1) / (high - low)))


def uint_to_float(value: int, low: float, high: float, bits: int) -> float:
    return value * (high - low) / ((1 << bits) - 1) + low


def pack_mit(position: float, velocity: float, kp: float, kd: float,
             torque: float, pmax: float, vmax: float, tmax: float) -> bytes:
    p = float_to_uint(position, -pmax, pmax, 16)
    v = float_to_uint(velocity, -vmax, vmax, 12)
    k_p = float_to_uint(kp, 0.0, 500.0, 12)
    k_d = float_to_uint(kd, 0.0, 5.0, 12)
    t = float_to_uint(torque, -tmax, tmax, 12)
    return bytes((p >> 8, p & 0xFF, v >> 4, ((v & 0xF) << 4) | (k_p >> 8),
                  k_p & 0xFF, k_d >> 4, ((k_d & 0xF) << 4) | (t >> 8), t & 0xFF))


@dataclass(frozen=True)
class Feedback:
    motor_id: int
    state: int
    position_rad: float
    velocity_rad_s: float
    torque_nm: float
    mos_c: int
    rotor_c: int


def unpack_feedback(data: bytes, pmax: float, vmax: float, tmax: float) -> Feedback:
    if len(data) != 8:
        raise ValueError("feedback must contain exactly 8 bytes")
    motor_id, state = data[0] & 0x0F, data[0] >> 4
    p = (data[1] << 8) | data[2]
    v = (data[3] << 4) | (data[4] >> 4)
    t = ((data[4] & 0x0F) << 8) | data[5]
    return Feedback(motor_id, state,
                    uint_to_float(p, -pmax, pmax, 16),
                    uint_to_float(v, -vmax, vmax, 12),
                    uint_to_float(t, -tmax, tmax, 12), data[6], data[7])


class MITController:
    """Convenience wrapper for MIT-mode position control with fixed gains."""

    def __init__(self, motor: object, kp: float, kd: float) -> None:
        self.motor = motor
        self.kp = kp
        self.kd = kd

    def move_position(self, target: float, torque_ff: float = 0.0) -> None:
        self.motor.command(  # type: ignore[attr-defined]
            position=target,
            velocity=0,
            kp=self.kp,
            kd=self.kd,
            torque=torque_ff,
        )


class DirectionGuard:
    """Latch the command direction that caused an excessive absolute torque."""

    def __init__(self, limit_nm: float, release_nm: float) -> None:
        if not 0.0 < release_nm < limit_nm:
            raise ValueError("release torque must be positive and below torque limit")
        self.limit_nm = limit_nm
        self.release_nm = release_nm
        self.blocked_direction = 0
        self.limit_position_rad: Optional[float] = None

    def observe(self, torque_nm: float, command_direction: int,
                position_rad: float) -> bool:
        if (self.blocked_direction == 0 and command_direction != 0
                and abs(torque_nm) >= self.limit_nm):
            self.blocked_direction = 1 if command_direction > 0 else -1
            self.limit_position_rad = position_rad
            return True
        return False

    def allows(self, direction: int) -> bool:
        return direction == 0 or direction != self.blocked_direction

    def unlock(self, torque_nm: float) -> bool:
        if abs(torque_nm) > self.release_nm:
            return False
        self.blocked_direction = 0
        self.limit_position_rad = None
        return True


# ---------------------------------------------------------------------------
# CAN backends
# ---------------------------------------------------------------------------

class CanMotor:
    """Linux SocketCAN backend."""

    def __init__(self, interface: str, motor_id: int, master_id: int,
                 pmax: float, vmax: float, tmax: float) -> None:
        self.motor_id, self.master_id = motor_id, master_id
        self.pmax, self.vmax, self.tmax = pmax, vmax, tmax
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((interface,))
        self.sock.setblocking(False)

    def send_can(self, can_id: int, payload: bytes) -> None:
        self.sock.send(CAN_FRAME.pack(can_id, len(payload), payload.ljust(8, b"\x00")))

    def send(self, payload: bytes) -> None:
        self.send_can(self.motor_id, payload)

    def command(self, position: float = 0.0, velocity: float = 0.0,
                kp: float = 0.0, kd: float = 0.0, torque: float = 0.0) -> None:
        self.send(pack_mit(position, velocity, kp, kd, torque,
                           self.pmax, self.vmax, self.tmax))

    def receive(self) -> Optional[Feedback]:
        try:
            can_id, dlc, data = CAN_FRAME.unpack(self.sock.recv(CAN_FRAME.size))
        except BlockingIOError:
            return None
        if (can_id & socket.CAN_EFF_MASK) != self.master_id or dlc < 8:
            return None
        feedback = unpack_feedback(data, self.pmax, self.vmax, self.tmax)
        return feedback if feedback.motor_id == (self.motor_id & 0x0F) else None

    def close(self) -> None:
        self.sock.close()


class PythonCanMotor:
    """Portable python-can transport, primarily for Windows USB-CAN adapters."""

    def __init__(self, interface: str, channel: str, motor_id: int, master_id: int,
                 pmax: float, vmax: float, tmax: float) -> None:
        try:
            import can  # type: ignore
        except ImportError as exc:
            raise RuntimeError(
                "Windows需要python-can；请运行: py -m pip install python-can"
            ) from exc
        parsed_channel: object = channel
        if interface == "gs_usb" and channel.isdecimal():
            parsed_channel = int(channel)
        self.can = can
        self.motor_id, self.master_id = motor_id, master_id
        self.pmax, self.vmax, self.tmax = pmax, vmax, tmax
        self.bus = can.Bus(interface=interface, channel=parsed_channel,
                           bitrate=1_000_000, ignore_config=True)

    def send_can(self, can_id: int, payload: bytes) -> None:
        self.bus.send(self.can.Message(arbitration_id=can_id,
                                       is_extended_id=False, data=payload), timeout=0.05)

    def send(self, payload: bytes) -> None:
        self.send_can(self.motor_id, payload)

    def command(self, position: float = 0.0, velocity: float = 0.0,
                kp: float = 0.0, kd: float = 0.0, torque: float = 0.0) -> None:
        self.send(pack_mit(position, velocity, kp, kd, torque,
                           self.pmax, self.vmax, self.tmax))

    def receive(self) -> Optional[Feedback]:
        message = self.bus.recv(timeout=0.0)
        if message is None or message.is_extended_id or message.arbitration_id != self.master_id:
            return None
        if len(message.data) != 8:
            return None
        feedback = unpack_feedback(bytes(message.data), self.pmax, self.vmax, self.tmax)
        return feedback if feedback.motor_id == (self.motor_id & 0x0F) else None

    def close(self) -> None:
        self.bus.shutdown()


class DamiaoSerialMotor:
    """DAMIAO USB2CAN CDC/CH340 transport (29-byte host protocol)."""

    REPORT_SIZE = 16

    def __init__(self, channel: str, motor_id: int, master_id: int,
                 pmax: float, vmax: float, tmax: float) -> None:
        try:
            import serial  # type: ignore
            from serial.tools import list_ports  # type: ignore
        except ImportError as exc:
            raise RuntimeError("USB串口后端需要pyserial：py -m pip install pyserial") from exc
        if channel == "auto":
            candidates = [p for p in list_ports.comports()
                          if p.vid in (0x1A86, 0x2E88)]
            if len(candidates) != 1:
                found = ", ".join(f"{p.device}({p.vid:04x}:{p.pid:04x})"
                                  for p in candidates) or "无"
                raise RuntimeError(f"无法唯一选择达妙USB串口；发现：{found}。请用--channel指定")
            channel = candidates[0].device
        if os.name != "nt" and channel.startswith("/") and not Path(channel).exists():
            raise RuntimeError(f"串口不存在：{channel}；请先检查 ls -l /dev/ttyUSB* /dev/ttyACM*")
        print(f"USB串口: {channel}（DAMIAO USB2CAN协议，CAN 1 Mbps）")
        self.motor_id, self.master_id = motor_id, master_id
        self.pmax, self.vmax, self.tmax = pmax, vmax, tmax
        try:
            import serial  # type: ignore
            self.serial = serial.Serial(channel, 921600, timeout=0, write_timeout=0.05)
        except serial.SerialException as exc:
            raise RuntimeError(f"无法打开串口 {channel}：{exc}") from exc
        self.buffer = bytearray()
        self.serial.write(b"\x55\x05\x00\xaa\x55")
        time.sleep(0.05)
        self.serial.reset_input_buffer()

    @staticmethod
    def host_frame(can_id: int, payload: bytes) -> bytes:
        if len(payload) != 8:
            raise ValueError("DAMIAO USB2CAN requires an 8-byte CAN payload")
        return struct.pack("<2sBBIIBIBBBB8sB", b"\x55\xaa", 0x1E, 0x03,
                           1, 10, 0, can_id, 0, 8, 0, 0, payload, 0)

    def send_can(self, can_id: int, payload: bytes) -> None:
        if len(payload) != 8:
            raise ValueError("DAMIAO serial backend requires 8-byte frames")
        self.serial.write(self.host_frame(can_id, payload))

    def send(self, payload: bytes) -> None:
        self.send_can(self.motor_id, payload)

    def command(self, position: float = 0.0, velocity: float = 0.0,
                kp: float = 0.0, kd: float = 0.0, torque: float = 0.0) -> None:
        self.send(pack_mit(position, velocity, kp, kd, torque,
                           self.pmax, self.vmax, self.tmax))

    def receive(self) -> Optional[Feedback]:
        waiting = self.serial.in_waiting
        if waiting:
            self.buffer.extend(self.serial.read(waiting))
        while True:
            start = self.buffer.find(b"\xaa")
            if start < 0:
                self.buffer.clear()
                return None
            if start:
                del self.buffer[:start]
            if len(self.buffer) < self.REPORT_SIZE:
                return None
            frame = bytes(self.buffer[:self.REPORT_SIZE])
            del self.buffer[:self.REPORT_SIZE]
            if frame[-1] != 0x55:
                continue
            cmd, flags, can_id = frame[1], frame[2], struct.unpack_from("<I", frame, 3)[0]
            if cmd != 0x11 or (flags & 0x3F) != 8 or can_id != self.master_id:
                continue
            feedback = unpack_feedback(frame[7:15], self.pmax, self.vmax, self.tmax)
            if feedback.motor_id == (self.motor_id & 0x0F):
                return feedback

    def close(self) -> None:
        self.serial.close()


class WaveshareSerialMotor:
    """Waveshare USB-CAN-A variable-length serial protocol."""

    def __init__(self, channel: str, motor_id: int, master_id: int,
                 pmax: float, vmax: float, tmax: float) -> None:
        try:
            import serial  # type: ignore
            from serial.tools import list_ports  # type: ignore
        except ImportError as exc:
            raise RuntimeError("微雪串口后端需要pyserial：py -m pip install pyserial") from exc
        if channel == "auto":
            candidates = [p for p in list_ports.comports() if p.vid == 0x1A86]
            if len(candidates) != 1:
                found = ", ".join(p.device for p in candidates) or "无"
                raise RuntimeError(f"无法唯一选择USB-CAN-A；发现：{found}。请用--channel指定")
            channel = candidates[0].device
        if os.name != "nt" and channel.startswith("/") and not Path(channel).exists():
            raise RuntimeError(f"串口不存在：{channel}")
        print(f"USB串口: {channel}（Waveshare USB-CAN-A，串口2 Mbps，CAN 1 Mbps）")
        try:
            import serial  # type: ignore
            self.serial = serial.Serial(channel, 2_000_000, timeout=0,
                                        write_timeout=0.05)
        except serial.SerialException as exc:
            raise RuntimeError(f"无法打开串口 {channel}：{exc}") from exc
        self.motor_id, self.master_id = motor_id, master_id
        self.pmax, self.vmax, self.tmax = pmax, vmax, tmax
        self.buffer = bytearray()
        self.last_register_response: Optional[tuple[int, int, bytes]] = None
        self.serial.reset_input_buffer()
        self.serial.write(self.settings_frame())
        self.serial.flush()
        time.sleep(0.10)
        self.serial.reset_input_buffer()

    @staticmethod
    def settings_frame() -> bytes:
        frame = bytearray((0xAA, 0x55, 0x12, 0x01, 0x01))
        frame.extend(b"\x00" * 8)
        frame.extend((0x00, 0x01, 0, 0, 0, 0))
        frame.append(sum(frame[2:19]) & 0xFF)
        return bytes(frame)

    @staticmethod
    def data_frame(can_id: int, payload: bytes) -> bytes:
        if not 0 <= can_id <= 0x7FF:
            raise ValueError("standard CAN ID must be in 0..0x7ff")
        if len(payload) > 8:
            raise ValueError("classic CAN payload cannot exceed 8 bytes")
        return bytes((0xAA, 0xC0 | len(payload), can_id & 0xFF,
                      (can_id >> 8) & 0xFF)) + payload + b"\x55"

    def send_can(self, can_id: int, payload: bytes) -> None:
        self.serial.write(self.data_frame(can_id, payload))

    def send(self, payload: bytes) -> None:
        self.send_can(self.motor_id, payload)

    def command(self, position: float = 0.0, velocity: float = 0.0,
                kp: float = 0.0, kd: float = 0.0, torque: float = 0.0) -> None:
        self.send(pack_mit(position, velocity, kp, kd, torque,
                           self.pmax, self.vmax, self.tmax))

    def receive(self) -> Optional[Feedback]:
        waiting = self.serial.in_waiting
        if waiting:
            self.buffer.extend(self.serial.read(waiting))
        while True:
            start = self.buffer.find(b"\xaa")
            if start < 0:
                self.buffer.clear()
                return None
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 2:
                return None
            if self.buffer[1] == 0x55:
                length = 20
            elif self.buffer[1] >> 4 == 0xC:
                length = (self.buffer[1] & 0x0F) + 5
            else:
                del self.buffer[0]
                continue
            if len(self.buffer) < length:
                return None
            frame = bytes(self.buffer[:length])
            del self.buffer[:length]
            if frame[-1] != 0x55 or frame[1] == 0x55:
                continue
            dlc = frame[1] & 0x0F
            can_id = frame[2] | (frame[3] << 8)
            if can_id != self.master_id:
                continue
            payload = frame[4:4 + dlc]
            if dlc == 4:
                addressed_id = payload[0] | (payload[1] << 8)
                if addressed_id == self.motor_id and payload[2] == 0xAA:
                    self.last_register_response = (payload[2], payload[3], b"")
                continue
            if dlc != 8:
                continue
            addressed_id = payload[0] | (payload[1] << 8)
            if (addressed_id == self.motor_id and payload[2] in (0x33, 0x55, 0xAA)):
                self.last_register_response = (payload[2], payload[3], payload[4:8])
                continue
            feedback = unpack_feedback(payload, self.pmax, self.vmax, self.tmax)
            if feedback.motor_id == (self.motor_id & 0x0F):
                return feedback

    def close(self) -> None:
        self.serial.close()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def wait_for_feedback(motor: object, timeout_s: float,
                      command_rate_hz: float) -> Feedback:
    deadline = time.monotonic() + timeout_s
    next_send = 0.0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_send:
            motor.command()  # type: ignore[attr-defined]
            next_send = now + 1.0 / command_rate_hz
        feedback = motor.receive()  # type: ignore[attr-defined]
        if feedback is not None and feedback.state == 1:
            return feedback
        time.sleep(0.001)
    raise RuntimeError("使能后没有收到电机反馈")


def set_control_mode(motor: object, mode: int, verify: bool = True) -> None:
    if mode not in (1, 2, 3, 4):
        raise ValueError("unsupported DM control mode")
    payload = struct.pack("<HBBI", motor.motor_id, 0x55, 0x0A, mode)  # type: ignore[attr-defined]
    if hasattr(motor, "last_register_response"):
        motor.last_register_response = None  # type: ignore[attr-defined]
    motor.send_can(0x7FF, payload)  # type: ignore[attr-defined]
    if verify and hasattr(motor, "last_register_response"):
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline:
            motor.receive()  # type: ignore[attr-defined]
            response = motor.last_register_response  # type: ignore[attr-defined]
            if response is not None:
                command, register, data = response
                value = struct.unpack("<I", data)[0]
                if command != 0x55 or register != 0x0A or value != mode:
                    raise RuntimeError(
                        f"CTRL_MODE回执不匹配：cmd={command:#x}, rid={register:#x}, value={value}")
                print(f"CTRL_MODE已确认切换为 {mode}")
                return
            time.sleep(0.001)
