"""PaXini GEN3 tactile sensor driver — HAND high-speed serial protocol.

Self-contained implementation of the binary protocol used by PaXini's
high-speed communication board.  No dependency on the official SDK.
Runtime dependency: pyserial.
"""
from __future__ import annotations

import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


BAUDRATE = 921_600

# Function codes
FUNC_READ = 0x03
FUNC_WRITE = 0x10
FUNC_CALIBRATE = 0x17

# Frame headers
REQUEST_HEAD = b"\x55\xAA"
RESPONSE_HEAD = b"\xAA\x55"
PUSH_HEAD = b"\xAA\x56"

# Registers
REG_VERSION = 0x0000
VERSION_DATA_LEN = 0x000F
REG_CONNECTED = 0x0010
REG_DATA_TYPE = 0x0016
REG_AUTO_PUSH = 0x0017
REG_CALIBRATE = 0x0002
REG_POINT_COUNTS = 0x0030
POINT_COUNTS_LEN = 0x0038

SCALE = 0.1  # raw integer → Newtons

SLOT_NAMES = (
    "大拇指近节", "大拇指中节", "大拇指指尖", "大拇指指甲",
    "食指近节", "食指中节", "食指指尖", "食指指甲",
    "中指近节", "中指中节", "中指指尖", "中指指甲",
    "无名指近节", "无名指中节", "无名指指尖", "无名指指甲",
    "小拇指近节", "小拇指中节", "小拇指指尖", "小拇指指甲",
    "掌心1", "掌心2", "掌心3", "掌心4", "掌心5", "掌心6", "掌心7", "掌心8",
)


class ProtocolError(RuntimeError):
    pass


# ---------------------------------------------------------------------------
# Low-level protocol helpers
# ---------------------------------------------------------------------------

def lrc(data: bytes) -> int:
    return (-sum(data)) & 0xFF


def build_request(func: int, address: int, length: int,
                  data: bytes = b"") -> bytes:
    body = (
        REQUEST_HEAD
        + b"\x00"
        + bytes((func,))
        + address.to_bytes(2, "little")
        + length.to_bytes(2, "little")
        + data
    )
    return body + bytes((lrc(body),))


def _signed_byte(b: int) -> int:
    return b if b <= 127 else b - 256


# ---------------------------------------------------------------------------
# Data parsers
# ---------------------------------------------------------------------------

def parse_force(data: bytes) -> tuple[int, int, int]:
    """6-byte resultant force → (Fx, Fy, Fz) raw."""
    if len(data) != 6:
        raise ProtocolError(f"合力应为6字节，实际{len(data)}字节")
    return _signed_byte(data[0]), _signed_byte(data[2]), data[4]


def parse_taxel(data: bytes) -> tuple[int, int, int]:
    """3-byte distributed taxel → (Fx, Fy, Fz) raw."""
    if len(data) != 3:
        raise ProtocolError(f"taxel应为3字节，实际{len(data)}字节")
    return _signed_byte(data[0]), _signed_byte(data[1]), data[2]


def parse_point_counts(data: bytes) -> list[int]:
    if len(data) != POINT_COUNTS_LEN:
        raise ProtocolError(
            f"测点数量表应为{POINT_COUNTS_LEN}字节，实际{len(data)}字节")
    return [int.from_bytes(data[i:i + 2], "little")
            for i in range(0, len(data), 2)]


def connected_slots(mask: bytes) -> list[int]:
    if len(mask) != 4:
        raise ProtocolError(f"连接状态应为4字节，实际{len(mask)}字节")
    value = int.from_bytes(mask, "little")
    return [slot for slot in range(28) if value & (1 << slot)]


# ---------------------------------------------------------------------------
# Serial frame I/O
# ---------------------------------------------------------------------------

def _send(ser, request: bytes) -> None:
    ser.reset_output_buffer()
    written = ser.write(request)
    ser.flush()
    if written != len(request):
        raise ProtocolError(
            f"请求发送不完整：应发{len(request)}字节，实发{written}字节")


def _read_frame(ser, head: bytes, timeout: float) -> bytes:
    buf = bytearray()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        waiting = ser.in_waiting
        chunk = ser.read(waiting or 1)
        if chunk:
            buf.extend(chunk)
        pos = buf.find(head)
        if pos < 0:
            if len(buf) > 1:
                del buf[:-1]
            continue
        if pos:
            del buf[:pos]
        if head == RESPONSE_HEAD:
            if len(buf) < 8:
                continue
            total = 9 + int.from_bytes(buf[6:8], "little")
        else:
            if len(buf) < 5:
                continue
            total = 6 + int.from_bytes(buf[3:5], "little")
        if len(buf) >= total:
            frame = bytes(buf[:total])
            if lrc(frame[:-1]) != frame[-1]:
                raise ProtocolError(f"{head.hex().upper()}响应LRC错误")
            return frame
    raise ProtocolError(f"等待{head.hex().upper()}响应超时")


def _parse_response(response: bytes) -> dict:
    if len(response) < 9 or response[:2] != RESPONSE_HEAD:
        raise ProtocolError("响应帧格式错误")
    data_len = int.from_bytes(response[6:8], "little")
    if len(response) != 9 + data_len:
        raise ProtocolError("响应帧长度错误")
    func = response[3]
    return {
        "is_error": bool(func & 0x80),
        "func_code": func,
        "reg_addr": int.from_bytes(response[4:6], "little"),
        "data_len": data_len,
        "data": response[8:-1],
    }


# ---------------------------------------------------------------------------
# Register read / write with retry
# ---------------------------------------------------------------------------

def read_register(ser, address: int, length: int,
                  timeout: float = 4.0) -> bytes:
    request = build_request(FUNC_READ, address, length)
    _send(ser, request)
    time.sleep(0.2)
    deadline = time.monotonic() + timeout
    attempts = 1
    while time.monotonic() < deadline:
        try:
            resp = _read_frame(
                ser, RESPONSE_HEAD,
                min(0.5, max(0.01, deadline - time.monotonic())))
        except ProtocolError as exc:
            if "超时" not in str(exc):
                raise
            if attempts < 5:
                _send(ser, request)
                attempts += 1
                time.sleep(0.2)
            continue
        parsed = _parse_response(resp)
        if (not parsed["is_error"]
                and parsed["func_code"] == FUNC_READ
                and parsed["reg_addr"] == address
                and parsed["data_len"] == length
                and len(parsed["data"]) == length):
            return parsed["data"]
        if attempts < 5:
            _send(ser, request)
            attempts += 1
            time.sleep(0.2)
    raise ProtocolError(f"等待寄存器0x{address:04X}匹配响应超时")


def write_register(ser, address: int, data: bytes,
                   timeout: float = 4.0) -> None:
    request = build_request(FUNC_WRITE, address, len(data), data)
    _send(ser, request)
    time.sleep(0.2)
    deadline = time.monotonic() + timeout
    attempts = 1
    while time.monotonic() < deadline:
        try:
            resp = _read_frame(
                ser, RESPONSE_HEAD,
                min(0.5, max(0.01, deadline - time.monotonic())))
        except ProtocolError as exc:
            if "超时" not in str(exc):
                raise
            if attempts < 5:
                _send(ser, request)
                attempts += 1
                time.sleep(0.2)
            continue
        parsed = _parse_response(resp)
        if (not parsed["is_error"]
                and parsed["func_code"] == FUNC_WRITE
                and parsed["reg_addr"] == address):
            if parsed["data"] and int.from_bytes(parsed["data"], "little"):
                raise ProtocolError(
                    f"写寄存器0x{address:04X}返回非零状态")
            return
        if attempts < 5:
            _send(ser, request)
            attempts += 1
            time.sleep(0.2)
    raise ProtocolError(f"等待写寄存器0x{address:04X}匹配响应超时")


# ---------------------------------------------------------------------------
# Calibration (firmware zero)
# ---------------------------------------------------------------------------

def firmware_zero(ser, timeout: float = 4.0) -> None:
    """Send the PaXini calibration command to zero all connected modules."""
    ser.reset_input_buffer()
    request = build_request(FUNC_CALIBRATE, REG_CALIBRATE, 1, b"\x01")
    _send(ser, request)
    resp = _read_frame(ser, RESPONSE_HEAD, timeout)
    parsed = _parse_response(resp)
    if (parsed["is_error"]
            or parsed["func_code"] != FUNC_CALIBRATE
            or parsed["reg_addr"] != REG_CALIBRATE
            or parsed["data"] != b"\x00"):
        raise ProtocolError(
            f"标定响应异常：func=0x{parsed['func_code']:02X}, "
            f"addr=0x{parsed['reg_addr']:04X}, "
            f"data={parsed['data'].hex()}")


# ---------------------------------------------------------------------------
# Port auto-detection
# ---------------------------------------------------------------------------

def choose_port(requested: Optional[str] = None) -> str:
    if requested and requested != "auto":
        return requested
    try:
        from serial.tools import list_ports
    except ImportError as exc:
        raise RuntimeError("pyserial未安装") from exc
    ports = list(list_ports.comports())
    paxini = [p for p in ports
              if "paxini" in (p.description or "").lower()]
    if len(paxini) == 1:
        return paxini[0].device
    if not paxini:
        raise RuntimeError("未发现PaXini高速板串口，请指定端口")
    raise RuntimeError("发现多个PaXini串口，请指定端口")


# ---------------------------------------------------------------------------
# Push reader
# ---------------------------------------------------------------------------

class PushReader:
    """Reads and validates AA56 auto-push frames from the serial buffer."""

    def __init__(self, ser) -> None:
        self.ser = ser
        self.buffer = bytearray()

    def read(self, timeout: float = 1.0) -> bytes:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            waiting = self.ser.in_waiting
            chunk = self.ser.read(waiting or 1)
            if chunk:
                self.buffer.extend(chunk)
            pos = self.buffer.find(PUSH_HEAD)
            if pos < 0:
                if len(self.buffer) > 1:
                    del self.buffer[:-1]
                continue
            if pos:
                del self.buffer[:pos]
            if len(self.buffer) < 5:
                continue
            payload_len = int.from_bytes(self.buffer[3:5], "little")
            total = 6 + payload_len
            if len(self.buffer) < total:
                continue
            frame = bytes(self.buffer[:total])
            del self.buffer[:total]
            if lrc(frame[:-1]) != frame[-1]:
                raise ProtocolError("AA56自动回传帧LRC错误")
            return frame
        raise ProtocolError("等待AA56自动回传帧超时")

    def clear(self) -> None:
        self.buffer.clear()


# ---------------------------------------------------------------------------
# Data containers
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class SlotReading:
    slot: int
    name: str
    fx_n: float
    fy_n: float
    fz_n: float
    taxels: tuple[tuple[float, float, float], ...] = ()


@dataclass(frozen=True)
class TactileFrame:
    timestamp: float
    slots: tuple[SlotReading, ...]

    def fz_by_slot(self, slot: int) -> float:
        for s in self.slots:
            if s.slot == slot:
                return s.fz_n
        return 0.0


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

class PaxiniSensor:
    """High-level PaXini GEN3 tactile sensor driver.

    Opens the serial port, performs handshake (version, connected slots,
    point counts), calibrates, configures data type, and starts auto-push.
    Call ``read()`` in a loop to get force readings, ``calibrate()`` for
    firmware zeroing, and ``close()`` to shut down cleanly.
    """

    def __init__(self, port: str = "auto",
                 left_slot: int = 5, right_slot: int = 8,
                 data_type: int = 0x01) -> None:
        try:
            import serial as _serial
        except ImportError as exc:
            raise RuntimeError("pyserial未安装") from exc

        self.left_slot = left_slot
        self.right_slot = right_slot
        self._data_type = data_type
        self._push_enabled = False
        self._ser: Optional[_serial.Serial] = None

        resolved = choose_port(port)
        if (os.name != "nt" and resolved.startswith("/")
                and not Path(resolved).exists()):
            raise RuntimeError(f"串口不存在：{resolved}")

        self._ser = _serial.Serial(
            port=resolved,
            baudrate=BAUDRATE,
            bytesize=_serial.EIGHTBITS,
            parity=_serial.PARITY_NONE,
            stopbits=_serial.STOPBITS_ONE,
            timeout=1,
            write_timeout=0.5,
            inter_byte_timeout=0.001,
        )

        # A previous process may have died while auto-push was active; the
        # handshake queries below would drown in stale push frames.
        try:
            _send(self._ser, build_request(FUNC_WRITE, REG_AUTO_PUSH, 1, b"\x00"))
            time.sleep(0.2)
        except Exception:
            pass
        self._ser.reset_input_buffer()

        last_error: Optional[Exception] = None
        for _attempt in range(3):
            try:
                version_data = read_register(
                    self._ser, REG_VERSION, VERSION_DATA_LEN)
                self.version = version_data.decode(
                    "ascii", errors="replace").rstrip("\x00 ")

                mask = read_register(self._ser, REG_CONNECTED, 4)
                self.slots = connected_slots(mask)

                pc_data = read_register(
                    self._ser, REG_POINT_COUNTS, POINT_COUNTS_LEN)
                all_counts = parse_point_counts(pc_data)
                self.point_counts = {s: all_counts[s] for s in self.slots}
                break
            except Exception as exc:
                last_error = exc
                self._ser.reset_input_buffer()
                time.sleep(0.15)
        else:
            self._ser.close()
            self._ser = None
            raise ProtocolError(f"PaXini握手重试失败：{last_error}")

        firmware_zero(self._ser)

        write_register(self._ser, REG_DATA_TYPE, bytes((data_type,)))

        self._enable_push()
        self._reader = PushReader(self._ser)

    # -- push control -------------------------------------------------------

    def _enable_push(self) -> None:
        request = build_request(FUNC_WRITE, REG_AUTO_PUSH, 1, b"\x01")
        _send(self._ser, request)
        resp = _read_frame(self._ser, PUSH_HEAD, 1.5)
        if len(resp) > 5 and resp[5] != 0:
            raise ProtocolError(
                f"开启自动回传失败，错误码0x{resp[5]:02X}")
        self._push_enabled = True

    def _disable_push(self) -> None:
        request = build_request(FUNC_WRITE, REG_AUTO_PUSH, 1, b"\x00")
        _send(self._ser, request)
        time.sleep(0.1)
        self._push_enabled = False

    # -- reading ------------------------------------------------------------

    def read(self, timeout: float = 1.0) -> TactileFrame:
        """Read one auto-push frame and return parsed force data."""
        frame = self._reader.read(timeout)
        error_code = frame[5]
        payload = frame[6:-1]
        if not payload:
            return self.read(timeout)
        if error_code:
            raise ProtocolError(
                f"高速板自动回传错误码0x{error_code:02X}")

        now = time.monotonic()
        readings = []
        offset = 0
        for slot in self.slots:
            fx, fy, fz = parse_force(payload[offset:offset + 6])
            offset += 6
            taxels: list[tuple[float, float, float]] = []
            if self._data_type & 0x02:
                count = self.point_counts.get(slot, 0)
                for _ in range(count):
                    tx, ty, tz = parse_taxel(payload[offset:offset + 3])
                    offset += 3
                    taxels.append((tx * SCALE, ty * SCALE, tz * SCALE))
            readings.append(SlotReading(
                slot=slot,
                name=SLOT_NAMES[slot] if slot < len(SLOT_NAMES) else f"slot{slot}",
                fx_n=fx * SCALE,
                fy_n=fy * SCALE,
                fz_n=fz * SCALE,
                taxels=tuple(taxels),
            ))
        return TactileFrame(timestamp=now, slots=tuple(readings))

    def read_normal_forces(self, timeout: float = 1.0) -> tuple[float, float]:
        """Convenience: return (left_fz_n, right_fz_n)."""
        frame = self.read(timeout)
        return (frame.fz_by_slot(self.left_slot),
                frame.fz_by_slot(self.right_slot))

    # -- calibration --------------------------------------------------------

    def calibrate(self) -> None:
        """Firmware zero: pause auto-push, zero all modules, resume."""
        was_pushing = self._push_enabled
        if was_pushing:
            self._disable_push()
        self._ser.reset_input_buffer()
        self._reader.clear()
        try:
            firmware_zero(self._ser)
        finally:
            if was_pushing:
                self._enable_push()

    # -- lifecycle ----------------------------------------------------------

    def close(self) -> None:
        if self._ser and self._ser.is_open:
            if self._push_enabled:
                try:
                    self._disable_push()
                except Exception:
                    pass
            self._ser.close()
        self._ser = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def __repr__(self) -> str:
        slots_str = ", ".join(
            f"{s}:{SLOT_NAMES[s]}" if s < len(SLOT_NAMES) else str(s)
            for s in self.slots)
        return (f"PaxiniSensor(version={self.version!r}, "
                f"slots=[{slots_str}])")
