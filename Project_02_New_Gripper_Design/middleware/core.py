"""Middleware: thread-safe state, hardware monitors, and the open/close flow.

Bridges the hardware drivers (DM-H3510 MIT-mode motor, dual PaXini GEN3
tactile modules) with the gripper open/close sequence:

    张开找零 → 无载触觉清零与验证 → 闭合至接触 → 张开 → 失能

方向约定（仅针对本装配方向）：正位置/正力矩为张开，负位置/负力矩为闭合。

边界（不是安全认证）：
* 接触停止阈值默认 0.5 N；25 N 是本流程写死的软件保护阈值，并非通用安全值。
* PMAX 自动核对只对 waveshare_serial 后端执行，VMAX/TMAX 未自动核对。
* 找零函数不检查触觉锁存；其位置误差限幅未计入 Kd 项，
  因此 home_torque 不能视为总扭矩硬限幅。
* 找零以力矩快速上升认定限位，异常摩擦/卡滞可能被误认作机械零点。
* 失能请求依赖通讯；零扭矩不等于主动卸载或确保机构静止。
* 空载闭合依靠左右手指互触触发接触检测；若行程内无接触会报错退出。
"""
from __future__ import annotations

import csv
import math
import struct
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Optional

from hardware.motor import (
    CanMotor,
    DamiaoSerialMotor,
    DISABLE,
    ENABLE,
    ERRORS,
    PythonCanMotor,
    WaveshareSerialMotor,
)
from hardware.paxini import PaxiniSensor


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

@dataclass
class MotorConfig:
    backend: str = "waveshare_serial"   # auto → waveshare_serial
    channel: str = "auto"
    interface: str = "can0"
    motor_id: int = 1
    master_id: int = 0
    pmax: float = 30.0      # 必须与电机 PMAX 寄存器一致；本机已持久化为 30 rad
    vmax: float = 280.0
    tmax: float = 1.0
    rate: float = 1000.0
    lead_mm: float = 4.0
    stroke_m: float = 0.01592   # slider_joint 全开到全闭直线行程
    feedback_timeout: float = 0.5
    enable_timeout: float = 3.0
    # 找零（张开方向 MIT 位置探索）
    home_torque: float = 0.40
    home_position_kp: float = 5.0
    home_position_kd: float = 0.05
    home_position_margin: float = 0.05
    home_feedforward: float = 0.35
    home_position_rate: float = 2.0
    home_expected_open_position: Optional[float] = None
    home_end_slow_distance: float = 0.60
    home_end_slow_rate: float = 0.25
    home_end_feedforward: float = 0.25
    home_end_threshold_distance: float = 0.15
    home_end_contact_torque: float = 0.08
    home_position_brake_torque: float = 0.18
    home_min_travel: float = 0.05
    home_already_open_delay: float = 1.0
    home_contact_torque: float = 0.10
    home_contact_speed: float = 0.20
    home_contact_samples: int = 3
    home_timeout: float = 180.0
    # 开合循环
    cycle_feedforward: float = 0.35
    cycle_kp: float = 5.0
    cycle_kd: float = 0.05
    cycle_torque_limit: float = 0.40
    cycle_closing_torque_limit: Optional[float] = None   # None → 等于 cycle_torque_limit
    cycle_tolerance: float = 0.03
    cycle_open_overshoot_limit: float = 0.15
    cycle_open_slow_distance: float = 0.60
    cycle_open_slow_speed: float = 0.25
    cycle_open_retract_margin: float = 0.10
    cycle_leg_timeout: float = 30.0
    allow_no_contact: bool = True   # 空载验证：走满闭合行程无接触也算正常完成
    contact_zero_tolerance: float = 0.3
    contact_force: float = 0.5
    approach_rate: float = 2.0
    output_dir: Path = Path(__file__).resolve().parent.parent / "hardware" / "experiment_results"


@dataclass
class TactileConfig:
    port: str = "auto"
    left_slot: int = 5
    right_slot: int = 8
    force_limit: float = 25.0     # 闭合时任一侧法向合力安全阈值 N
    closing_velocity: float = 0.01
    closing_hold_time: float = 1.0


def validate_configs(motor: MotorConfig, tactile: TactileConfig) -> None:
    for name, value in vars(motor).items():
        if isinstance(value, float) and not math.isfinite(value):
            raise ValueError(f"{name} 必须为有限值")
    positive = ("lead_mm", "stroke_m", "pmax", "vmax", "tmax", "rate",
                "feedback_timeout", "enable_timeout", "home_position_kp",
                "home_position_rate", "home_timeout", "home_torque",
                "home_position_brake_torque", "home_contact_torque",
                "home_contact_samples", "cycle_kp", "cycle_torque_limit",
                "cycle_leg_timeout", "cycle_tolerance",
                "cycle_open_overshoot_limit", "approach_rate")
    if any(getattr(motor, key) <= 0 for key in positive):
        raise ValueError("时间、量程、增益、速度、力矩上限和容差必须为正")
    closing_limit = motor.cycle_closing_torque_limit
    if closing_limit is None:
        closing_limit = motor.cycle_torque_limit
    if closing_limit <= 0:
        raise ValueError("闭合力矩上限必须为正")
    if not 0 <= motor.contact_zero_tolerance < motor.contact_force < tactile.force_limit:
        raise ValueError("要求 0 ≤ 零点容差 < 接触阈值 < 安全硬上限")
    if not 0 <= motor.cycle_feedforward < min(motor.cycle_torque_limit, closing_limit):
        raise ValueError("前馈必须非负且小于两个方向的力矩上限")
    if not (0 <= motor.home_end_feedforward < motor.home_torque and
            0 <= motor.home_feedforward < motor.home_torque and
            0 < motor.home_end_contact_torque < motor.home_contact_torque and
            0 < motor.home_end_slow_rate <= motor.home_position_rate and
            0 < motor.home_end_threshold_distance <= motor.home_end_slow_distance and
            0 < motor.cycle_open_retract_margin < motor.cycle_open_slow_distance and
            motor.cycle_open_slow_speed > 0 and
            0 < motor.home_position_margin < motor.pmax and
            motor.cycle_kd >= 0 and motor.home_position_kd >= 0):
        raise ValueError("找零、减速或限幅参数无效")
    if max(motor.home_torque, motor.home_position_brake_torque,
           motor.cycle_torque_limit, closing_limit) > motor.tmax:
        raise ValueError("请求的力矩上限不能超过 MIT 编码 TMAX")
    if tactile.left_slot == tactile.right_slot:
        raise ValueError("左右传感器 slot 必须不同")


# ---------------------------------------------------------------------------
# Gripper model: lead-screw slider kinematics
# ---------------------------------------------------------------------------

class GripperModel:
    """Lead-screw slider model: motor angle ↔ slider joint position.

    正电机位置对应张开；滑块位置从 upper（全开）向 lower（全闭）减小。
    """

    def __init__(self, lead_m: float, stroke_m: float) -> None:
        self.lead_m = lead_m
        self.lower = 0.0
        self.upper = stroke_m

    @property
    def closed_travel_rad(self) -> float:
        return (self.upper - self.lower) * 2 * math.pi / self.lead_m

    def motor_to_slider(self, position_rad: float,
                        open_zero_rad: float) -> tuple[float, float, bool]:
        raw = self.lower - (position_rad - open_zero_rad) * self.lead_m / (2 * math.pi)
        value = max(self.lower, min(self.upper, raw))
        open_percent = 100.0 * (self.upper - value) / (self.upper - self.lower)
        return value, open_percent, raw != value


# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------

class SharedState:
    """Thread-safe container for the latest gripper and tactile state."""

    def __init__(self, model: GripperModel) -> None:
        self.model = model
        self.open_zero = 0.0
        self._lock = threading.Lock()
        self.tactile_calibration_request = threading.Event()
        self.tactile_calibration_done = threading.Event()
        self.tactile_calibration_error = ""
        self._gripper: dict[str, Any] = {
            "connected": False, "error": "waiting", "homed": False,
            "homing": False, "open_zero_rad": 0.0,
            "position_rad": 0.0, "velocity_rad_s": 0.0, "torque_nm": 0.0,
            "slider_joint_m": model.upper, "open_percent": 100.0,
            "outside_urdf_range": False, "mos_c": 0, "rotor_c": 0,
        }
        self._tactile: dict[str, Any] = {
            "connected": False, "error": "waiting", "fps": 0.0,
            "left_normal_n": 0.0, "right_normal_n": 0.0,
            "sample_timestamp": 0.0,
        }
        self._safety: dict[str, Any] = {
            "latched": False, "reason": "", "tripped_at": None,
        }

    # -- gripper ------------------------------------------------------------

    def set_open_zero(self, position_rad: float) -> None:
        with self._lock:
            self.open_zero = position_rad
            self._gripper["homed"] = True
            self._gripper["open_zero_rad"] = position_rad
            self._gripper["homing"] = False

    def set_homing(self, active: bool) -> None:
        with self._lock:
            self._gripper["homing"] = active

    def feedback(self, feedback: Any) -> None:
        slider, percent, clipped = self.model.motor_to_slider(
            feedback.position_rad, self.open_zero)
        with self._lock:
            self._gripper.update(
                connected=True, error="", **asdict(feedback),
                slider_joint_m=slider, open_percent=percent,
                outside_urdf_range=clipped)

    def error(self, message: str) -> None:
        with self._lock:
            self._gripper.update(connected=False, error=message)

    def update_gripper(self, **kw: Any) -> None:
        with self._lock:
            self._gripper.update(kw)

    # -- tactile ------------------------------------------------------------

    def tactile(self, left_normal_n: float, right_normal_n: float,
                fps: float) -> None:
        with self._lock:
            self._tactile.update(
                connected=True, error="", fps=fps,
                left_normal_n=left_normal_n, right_normal_n=right_normal_n,
                sample_timestamp=time.monotonic())

    def tactile_error(self, message: str) -> None:
        with self._lock:
            self._tactile.update(connected=False, error=message)

    def update_tactile(self, **kw: Any) -> None:
        with self._lock:
            self._tactile.update(kw)

    def request_tactile_calibration(self) -> None:
        self.tactile_calibration_error = ""
        self.tactile_calibration_done.clear()
        self.tactile_calibration_request.set()

    # -- safety -------------------------------------------------------------

    def latch_safety(self, reason: str) -> None:
        with self._lock:
            if not self._safety["latched"]:
                self._safety.update(
                    latched=True, reason=reason, tripped_at=time.time())

    def safety_latched(self) -> bool:
        with self._lock:
            return bool(self._safety["latched"])

    def snapshot(self) -> dict[str, dict[str, Any]]:
        with self._lock:
            return {
                "gripper": dict(self._gripper),
                "tactile": dict(self._tactile),
                "safety": dict(self._safety),
            }


# ---------------------------------------------------------------------------
# Motor backend factory
# ---------------------------------------------------------------------------

_BACKENDS = {
    "socketcan": lambda cfg: CanMotor(
        cfg.interface, cfg.motor_id, cfg.master_id, cfg.pmax, cfg.vmax, cfg.tmax),
    "waveshare_serial": lambda cfg: WaveshareSerialMotor(
        cfg.channel, cfg.motor_id, cfg.master_id, cfg.pmax, cfg.vmax, cfg.tmax),
    "damiao_serial": lambda cfg: DamiaoSerialMotor(
        cfg.channel, cfg.motor_id, cfg.master_id, cfg.pmax, cfg.vmax, cfg.tmax),
}


def _create_motor(cfg: MotorConfig):
    backend = "waveshare_serial" if cfg.backend == "auto" else cfg.backend
    factory = _BACKENDS.get(backend)
    if factory:
        return factory(cfg)
    return PythonCanMotor(
        backend, cfg.channel, cfg.motor_id, cfg.master_id,
        cfg.pmax, cfg.vmax, cfg.tmax)


# ---------------------------------------------------------------------------
# Motor monitor: enable, home, open/close cycle
# ---------------------------------------------------------------------------

class MotorMonitor(threading.Thread):
    """Creates and enables the motor, then runs the open/close sequence."""

    def __init__(self, state: SharedState, config: MotorConfig,
                 tactile_config: TactileConfig) -> None:
        super().__init__(daemon=True)
        self.state = state
        self.config = config
        self.tactile_config = tactile_config
        self.motor: Optional[Any] = None
        self._stop_event = threading.Event()

    def stop(self) -> None:
        self._stop_event.set()

    def run(self) -> None:
        try:
            self.motor = _create_motor(self.config)
            if self.config.backend in ("auto", "waveshare_serial"):
                self.verify_pmax_before_enable()
            initial = self.enable_and_wait()
            self.state.feedback(initial)
            self.run_open_close_cycle(initial)
        except Exception as exc:
            self.state.error(str(exc))
            print(f"电机线程错误：{exc}", flush=True)
        finally:
            if self.motor is not None:
                try:
                    try:
                        self.motor.command()
                    finally:
                        self.motor.send(DISABLE)
                finally:
                    self.motor.close()

    # -- enable -------------------------------------------------------------

    def verify_pmax_before_enable(self) -> None:
        """Read PMAX while disabled; an MIT range mismatch can cause flyaway."""
        assert self.motor is not None
        self.motor.send(bytes((0xFF,) * 7 + (0xFD,)))
        time.sleep(0.05)
        self.motor.last_register_response = None
        payload = struct.pack("<HBB", self.config.motor_id, 0x33, 0x15) + bytes(4)
        self.motor.send_can(0x7FF, payload)
        deadline = time.monotonic() + 0.8
        while time.monotonic() < deadline:
            self.motor.receive()
            response = self.motor.last_register_response
            if response is not None:
                command, rid, data = response
                self.motor.last_register_response = None
                if command == 0x33 and rid == 0x15 and len(data) == 4:
                    hardware_pmax = struct.unpack("<f", data)[0]
                    if not math.isclose(hardware_pmax, self.config.pmax,
                                        rel_tol=0.0, abs_tol=1e-4):
                        raise RuntimeError(
                            "禁止使能：电机PMAX寄存器="
                            f"{hardware_pmax:.6f} rad，但软件pmax="
                            f"{self.config.pmax:.6f} rad；MIT位置编码量程不一致会飞车")
                    print(f"MIT位置量程已核对：硬件PMAX=软件PMAX="
                          f"{hardware_pmax:.6f} rad。", flush=True)
                    return
            time.sleep(0.001)
        raise RuntimeError("禁止使能：读取电机PMAX寄存器超时")

    def enable_and_wait(self) -> Any:
        """Require explicit state=enabled feedback before any homing torque."""
        assert self.motor is not None
        deadline = time.monotonic() + self.config.enable_timeout
        last_feedback = None
        next_enable = 0.0
        print("正在发送MIT使能并等待绿色状态灯……", flush=True)
        while time.monotonic() < deadline and not self._stop_event.is_set():
            now = time.monotonic()
            if now >= next_enable:
                self.motor.send(ENABLE)
                next_enable = now + 0.20
            # A neutral MIT frame elicits normal feedback on adapters that do
            # not return a status frame for the special enable command itself.
            self.motor.command()
            for _ in range(32):
                feedback = self.motor.receive()
                if feedback is None:
                    break
                last_feedback = feedback
                if feedback.state == 1:
                    print(f"电机已确认使能：state=enabled，位置 "
                          f"{feedback.position_rad:.6f} rad", flush=True)
                    return feedback
            time.sleep(0.01)
        if last_feedback is None:
            raise RuntimeError(
                "MIT使能失败：未收到电机反馈；检查CAN ID、USB-CAN及接线")
        raise RuntimeError(
            f"MIT使能失败：反馈状态={last_feedback.state}，"
            f"即{ERRORS.get(last_feedback.state, 'unknown')}；"
            "确认驱动器已设置为MIT模式")

    def current_enabled_feedback(self) -> Any:
        """Fetch a fresh post-homing feedback sample before starting a cycle."""
        assert self.motor is not None
        deadline = time.monotonic() + self.config.feedback_timeout
        while time.monotonic() < deadline and not self._stop_event.is_set():
            self.motor.command()
            for _ in range(32):
                feedback = self.motor.receive()
                if feedback is None:
                    break
                self.state.feedback(feedback)
                if feedback.state == 1:
                    return feedback
            time.sleep(0.002)
        raise RuntimeError("找零完成后无法取得新的使能反馈")

    # -- homing -------------------------------------------------------------

    def home_open_mit_position(self, initial: Any) -> None:
        """Slow positive MIT position probe with constant opening feedforward."""
        assert self.motor is not None
        cfg = self.config
        self.state.set_homing(True)
        started = last_feedback = time.monotonic()
        start_position = initial.position_rad
        latest = initial
        target = start_position
        target_time = started
        next_send = started
        consecutive = 0
        estimated_velocity = 0.0
        position_window: list[tuple[float, float]] = []
        safe_high = cfg.pmax - cfg.home_position_margin
        # The previous absolute position is only an approach landmark. The
        # mechanical open zero is still re-measured after every restart.
        expected_open = cfg.home_expected_open_position
        if expected_open is None and abs(self.state.open_zero) > 1.0:
            expected_open = self.state.open_zero
        print(f"开始MIT位置探索找零：前馈 +{cfg.home_feedforward:.3f} N·m，"
              f"位置推进 +{cfg.home_position_rate:.3f} rad/s，"
              f"Kp={cfg.home_position_kp:.3f}, Kd={cfg.home_position_kd:.3f}。")
        while not self._stop_event.is_set():
            now = time.monotonic()
            if now >= next_send:
                dt = max(0.0, min(0.05, now - target_time))
                target_time = now
                near_open = (expected_open is not None and
                             latest.position_rad >=
                             expected_open - cfg.home_end_slow_distance)
                command_rate = (cfg.home_end_slow_rate if near_open
                                else cfg.home_position_rate)
                command_feedforward = (cfg.home_end_feedforward if near_open
                                       else cfg.home_feedforward)
                target = min(safe_high, target + command_rate * dt)
                # Clip position error so Kp*error + feedforward stays bounded.
                min_error = (-cfg.home_position_brake_torque -
                             command_feedforward) / cfg.home_position_kp
                max_error = (cfg.home_torque -
                             command_feedforward) / cfg.home_position_kp
                target = max(latest.position_rad + min_error,
                             min(latest.position_rad + max_error, target))
                self.motor.command(position=target, velocity=0.0,
                                   kp=cfg.home_position_kp,
                                   kd=cfg.home_position_kd,
                                   torque=command_feedforward)
                next_send = now + 1 / cfg.rate
            for _ in range(32):
                feedback = self.motor.receive()
                if feedback is None:
                    break
                latest, last_feedback = feedback, now
                self.state.feedback(feedback)
                position_window.append((now, feedback.position_rad))
                while (len(position_window) > 2 and
                       now - position_window[0][0] > 0.05):
                    position_window.pop(0)
                window_dt = position_window[-1][0] - position_window[0][0]
                if window_dt >= 0.035:
                    estimated_velocity = ((position_window[-1][1] -
                                           position_window[0][1]) / window_dt)
                if feedback.state != 1:
                    raise RuntimeError(f"MIT位置找零电机状态异常：{feedback.state}")
                travel = abs(feedback.position_rad - start_position)
                at_stop = (window_dt >= 0.035 and
                           abs(estimated_velocity) <= cfg.home_contact_speed)
                eligible = (travel >= cfg.home_min_travel or
                            now - started >= cfg.home_already_open_delay)
                near_expected_open = (expected_open is not None and
                                      feedback.position_rad >=
                                      expected_open - cfg.home_end_threshold_distance)
                contact_threshold = (cfg.home_end_contact_torque
                                     if near_expected_open
                                     else cfg.home_contact_torque)
                # A hard-stop torque rise must neutralize immediately. Waiting
                # for the velocity estimate and several more frames allowed
                # the low-inertia gripper to push audibly into its end stop.
                if eligible and abs(feedback.torque_nm) >= contact_threshold:
                    self.motor.command()
                    self.state.set_open_zero(feedback.position_rad)
                    print(f"张开零点找到：{feedback.position_rad:.6f} rad；"
                          f"反馈扭矩 {feedback.torque_nm:.4f} N·m（快速保护触发）")
                    print("张开零点仅用于本次运行，不写入文件。")
                    return
                consecutive = (consecutive + 1 if eligible and at_stop and
                               abs(feedback.torque_nm) >= contact_threshold else 0)
                if consecutive >= cfg.home_contact_samples:
                    self.motor.command()
                    self.state.set_open_zero(feedback.position_rad)
                    print(f"张开零点找到：{feedback.position_rad:.6f} rad；"
                          f"反馈扭矩 {feedback.torque_nm:.4f} N·m")
                    print("张开零点仅用于本次运行，不写入文件。")
                    return
            if latest.position_rad >= safe_high:
                raise RuntimeError("到达+PMAX安全边界但未检测到机械限位")
            if now - last_feedback > cfg.feedback_timeout:
                raise RuntimeError("MIT位置找零反馈超时")
            if now - started > cfg.home_timeout:
                raise RuntimeError("MIT位置找零超过最大时间")
            time.sleep(0.001)
        raise RuntimeError("MIT位置找零被停止")

    # -- tactile recalibration ----------------------------------------------

    def recalibrate_tactile_after_open(self) -> None:
        assert self.motor is not None
        print("夹爪已完全张开，正在重新标定PaXini，请勿触碰……", flush=True)
        self.state.request_tactile_calibration()
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not self._stop_event.is_set():
            self.motor.command()
            if self.state.tactile_calibration_done.wait(0.02):
                if self.state.tactile_calibration_error:
                    raise RuntimeError(self.state.tactile_calibration_error)
                print("张开后的PaXini标定命令完成。", flush=True)
                return
        raise RuntimeError("等待张开后的PaXini标定超时")

    # -- open/close cycle -----------------------------------------------------

    def run_open_close_cycle(self, initial: Any) -> None:
        """One close-to-contact/reopen cycle; no force PID.

        空载时闭合段依靠左右手指互触触发接触阈值而停止。
        """
        assert self.motor is not None
        cfg = self.config
        output = cfg.output_dir
        output.mkdir(parents=True, exist_ok=True)
        path = output / ("open_close_" + time.strftime("%Y%m%d_%H%M%S") +
                         f"_{time.time_ns() % 1000000000:09d}.csv")
        started = time.monotonic()
        phase = "HOME"
        latest = initial
        last_feedback = started
        status = "aborted"
        with path.open("x", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(("time_s", "phase", "left_n", "right_n",
                             "sample_timestamp", "position_rad",
                             "velocity_rad_s", "command_position_rad",
                             "feedforward_nm", "motor_kp", "motor_kd", "event"))

            def sample():
                nonlocal latest, last_feedback
                now = time.monotonic()
                if self._stop_event.is_set():
                    raise RuntimeError("用户取消：停止并失能，不自动回程")
                for _ in range(32):
                    fb = self.motor.receive()
                    if fb is None:
                        break
                    latest = fb
                    last_feedback = now
                    self.state.feedback(fb)
                    if fb.state != 1:
                        raise RuntimeError(f"电机状态异常：{fb.state}")
                snapshot = self.state.snapshot()
                tac = snapshot["tactile"]
                age = now - tac.get("sample_timestamp", 0)
                if (not tac["connected"] or age > cfg.feedback_timeout or
                        now - last_feedback > cfg.feedback_timeout):
                    raise RuntimeError(
                        f"反馈异常：触觉年龄={age:.3f}s，错误={tac.get('error', '')}")
                force = max(tac["left_normal_n"], tac["right_normal_n"])
                if not all(math.isfinite(tac[k])
                           for k in ("left_normal_n", "right_normal_n")):
                    raise RuntimeError("触觉数值无效")
                if (force >= self.tactile_config.force_limit or
                        snapshot["safety"]["latched"]):
                    raise RuntimeError(
                        f"保护停止：左/右={tac['left_normal_n']}/"
                        f"{tac['right_normal_n']} N；{snapshot['safety']['reason']}")
                return now, tac, force

            def record(tac, position, ff, kp, kd, event=""):
                writer.writerow((time.monotonic() - started, phase,
                                 tac["left_normal_n"], tac["right_normal_n"],
                                 tac.get("sample_timestamp"),
                                 latest.position_rad, latest.velocity_rad_s,
                                 position, ff, kp, kd, event))
                stream.flush()

            def verify_zero():
                # Ignore the synthetic zero emitted by the calibration thread.
                after = time.monotonic()
                previous = after
                count = 0
                while time.monotonic() - after < 3.0:
                    self.motor.command()
                    now, tac, force = sample()
                    if tac["sample_timestamp"] > previous:
                        previous = tac["sample_timestamp"]
                        record(tac, latest.position_rad, 0, 0, 0)
                        count = (count + 1 if force <= cfg.contact_zero_tolerance
                                 else 0)
                        if count >= 10:
                            print(f"{phase}通过：10个新帧，左/右="
                                  f"{tac['left_normal_n']:.2f}/"
                                  f"{tac['right_normal_n']:.2f} N。", flush=True)
                            return
                    time.sleep(1 / cfg.rate)
                raise RuntimeError(f"{phase}未通过：左右力未持续回到零点容差内")

            try:
                print(f"开合验证：硬上限{self.tactile_config.force_limit:.0f} N；"
                      f"接触检测{cfg.contact_force:.2f} N；"
                      f"零点容差{cfg.contact_zero_tolerance:.2f} N。", flush=True)
                self.home_open_mit_position(latest)
                latest = self.current_enabled_feedback()
                self.recalibrate_tactile_after_open()
                latest = self.current_enabled_feedback()
                last_feedback = time.monotonic()
                phase = "ZERO_VERIFY"
                verify_zero()
                closed = self.state.open_zero - self.state.model.closed_travel_rad
                # Return inside the measured mechanical stop, without pressing it again.
                open_target = self.state.open_zero - cfg.cycle_open_retract_margin
                for opening in (False, True):
                    phase = "REOPEN" if opening else "APPROACH"
                    target = latest.position_rad
                    leg_start = previous_time = time.monotonic()
                    while True:
                        now, tac, force = sample()
                        if not opening and force >= cfg.contact_force:
                            self.motor.command()
                            record(tac, latest.position_rad, 0, 0, 0,
                                   "CONTACT_STOP")
                            print(f"检测接触：左/右={tac['left_normal_n']:.2f}/"
                                  f"{tac['right_normal_n']:.2f} N；"
                                  "停止闭合，开始受控张开。", flush=True)
                            break
                        if (opening and latest.position_rad >
                                self.state.open_zero + cfg.cycle_open_overshoot_limit):
                            raise RuntimeError("回程超出张开行程")
                        if (opening and latest.position_rad >=
                                open_target - cfg.cycle_tolerance):
                            self.motor.command()
                            record(tac, latest.position_rad, 0, 0, 0, "OPEN_STOP")
                            break
                        if not opening and latest.position_rad <= closed:
                            if cfg.allow_no_contact:
                                self.motor.command()
                                record(tac, latest.position_rad, 0, 0, 0,
                                       "CLOSED_TRAVEL_STOP")
                                print("空载闭合到位：走满闭合行程且无接触，"
                                      "开始受控张开。", flush=True)
                                break
                            raise RuntimeError("到达闭合行程，仍无接触")
                        if now - leg_start > cfg.cycle_leg_timeout:
                            raise RuntimeError(f"{phase}超时")
                        dt = min(0.05, max(0, now - previous_time))
                        previous_time = now
                        direction = 1 if opening else -1
                        near = (opening and
                                open_target - latest.position_rad <
                                cfg.cycle_open_slow_distance)
                        speed = (cfg.cycle_open_slow_speed if near
                                 else cfg.approach_rate)
                        target += direction * speed * dt
                        target = (min(open_target, target) if opening
                                  else max(closed, target))
                        ff = direction * cfg.cycle_feedforward
                        damping = -cfg.cycle_kd * latest.velocity_rad_s
                        closing_limit = (cfg.cycle_closing_torque_limit
                                         if cfg.cycle_closing_torque_limit is not None
                                         else cfg.cycle_torque_limit)
                        limit = min(cfg.cycle_torque_limit, closing_limit)
                        command = max(
                            latest.position_rad + (-limit - ff - damping) / cfg.cycle_kp,
                            min(latest.position_rad + (limit - ff - damping) / cfg.cycle_kp,
                                target))
                        self.motor.command(position=command, velocity=0,
                                           kp=cfg.cycle_kp, kd=cfg.cycle_kd,
                                           torque=ff)
                        record(tac, command, ff, cfg.cycle_kp, cfg.cycle_kd)
                        time.sleep(1 / cfg.rate)
                phase = "RETURN_ZERO_VERIFY"
                verify_zero()
                status = "completed"
            except Exception as exc:
                self.motor.command()
                writer.writerow((time.monotonic() - started, phase,
                                 "", "", "", "", "", "", "", "", "", str(exc)))
                stream.flush()
                raise
            finally:
                self.motor.command()
                self.motor.send(DISABLE)
                print(f"开合验证{status}，已请求失能。日志：{path}", flush=True)


# ---------------------------------------------------------------------------
# Tactile monitor
# ---------------------------------------------------------------------------

class TactileMonitor(threading.Thread):
    """Background thread: reads dual PaXini normal forces and watches safety."""

    def __init__(self, state: SharedState, config: TactileConfig) -> None:
        super().__init__(daemon=True)
        self.state = state
        self.config = config
        self.sensor: Optional[PaxiniSensor] = None
        self._stop_event = threading.Event()

    def stop(self) -> None:
        self._stop_event.set()

    def run(self) -> None:
        try:
            self.sensor = PaxiniSensor(
                port=self.config.port,
                left_slot=self.config.left_slot,
                right_slot=self.config.right_slot,
                data_type=0x03)
            required = {self.config.left_slot, self.config.right_slot}
            if not required.issubset(self.sensor.slots):
                raise RuntimeError(
                    f"PaXini需要slots {sorted(required)}，实际{self.sensor.slots}")
            frames, rate_start, fps = 0, time.monotonic(), 0.0
            last_closing_motion = float("-inf")
            while not self._stop_event.is_set():
                if self.state.tactile_calibration_request.is_set():
                    try:
                        self.sensor.calibrate()
                        self.state.tactile(0.0, 0.0, 0.0)
                    except Exception as exc:
                        self.state.tactile_calibration_error = (
                            f"张开后PaXini标定失败：{exc}")
                    finally:
                        self.state.tactile_calibration_request.clear()
                        self.state.tactile_calibration_done.set()
                    continue
                left_raw, right_raw = self.sensor.read_normal_forces(timeout=1.0)
                left, right = abs(left_raw), abs(right_raw)
                frames += 1
                now = time.monotonic()
                if now - rate_start >= 1:
                    fps, frames, rate_start = frames / (now - rate_start), 0, now
                self.state.tactile(left, right, fps)
                snapshot = self.state.snapshot()
                if (snapshot["gripper"]["velocity_rad_s"]
                        < -self.config.closing_velocity):
                    last_closing_motion = now
                # Contact can reduce velocity to zero before the force frame
                # arrives. Keep the closing classification briefly after
                # positive motion.
                closing = now - last_closing_motion <= self.config.closing_hold_time
                if closing and max(left, right) >= self.config.force_limit:
                    self.state.latch_safety(
                        f"PaXini {max(left, right):.2f} N ≥ "
                        f"{self.config.force_limit:.2f} N")
        except Exception as exc:
            self.state.tactile_error(str(exc))
            self.state.latch_safety(f"PaXini失联：{exc}")
        finally:
            if self.sensor is not None:
                self.sensor.close()
