#!/usr/bin/env python3
"""夹爪开合入口：张开找零 → 触觉清零验证 → 闭合至接触 → 张开 → 失能。

正位置/正力矩为张开，负位置/负力矩为闭合（仅针对本装配方向）。
空载验证时闭合段依靠左右手指互触触发 0.5 N 接触阈值而停止。

运行示例：
  python3 main.py --channel /dev/ttyUSB0 --paxini-port /dev/ttyACM0

边界（不是安全认证）：
* 0.5 N 是默认接触停止阈值；25 N 是写死的软件保护阈值，并非通用安全值。
* PMAX 自动核对只对 waveshare_serial 后端执行，VMAX/TMAX 未自动核对。
* 失能请求依赖通讯；零扭矩不等于主动卸载或确保机构静止。
"""
import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from middleware.core import (
    GripperModel,
    MotorConfig,
    MotorMonitor,
    SharedState,
    TactileConfig,
    TactileMonitor,
    validate_configs,
)


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="夹爪：张开找零、闭合至接触、重新张开")
    p.add_argument("--backend", default="auto",
                   choices=("auto", "waveshare_serial", "damiao_serial",
                            "socketcan", "gs_usb", "slcan", "pcan"))
    p.add_argument("--channel", default="auto")
    p.add_argument("--interface", default="can0")
    p.add_argument("--motor-id", type=lambda x: int(x, 0), default=1)
    p.add_argument("--master-id", type=lambda x: int(x, 0), default=0)
    p.add_argument("--pmax", type=float, default=30.0,
                   help="必须与电机PMAX寄存器一致；本机已持久化为30rad")
    p.add_argument("--vmax", type=float, default=280.0)
    p.add_argument("--tmax", type=float, default=1.0)
    p.add_argument("--rate", type=float, default=1000.0)
    p.add_argument("--lead-mm", type=float, default=4.0)
    p.add_argument("--stroke-mm", type=float, default=15.92,
                   help="slider_joint 全开到全闭直线行程 mm")
    p.add_argument("--feedback-timeout", type=float, default=0.5)
    p.add_argument("--enable-timeout", type=float, default=3.0)
    # 找零
    p.add_argument("--home-torque", type=float, default=0.40)
    p.add_argument("--home-position-kp", type=float, default=5.0)
    p.add_argument("--home-position-kd", type=float, default=0.05)
    p.add_argument("--home-position-margin", type=float, default=0.05)
    p.add_argument("--home-feedforward", type=float, default=0.35)
    p.add_argument("--home-position-rate", type=float, default=2.0)
    p.add_argument("--home-expected-open-position", type=float, default=None)
    p.add_argument("--home-end-slow-distance", type=float, default=0.60)
    p.add_argument("--home-end-slow-rate", type=float, default=0.25)
    p.add_argument("--home-end-feedforward", type=float, default=0.25)
    p.add_argument("--home-end-threshold-distance", type=float, default=0.15)
    p.add_argument("--home-end-contact-torque", type=float, default=0.08)
    p.add_argument("--home-position-brake-torque", type=float, default=0.18)
    p.add_argument("--home-min-travel", type=float, default=0.05)
    p.add_argument("--home-already-open-delay", type=float, default=1.0)
    p.add_argument("--home-contact-torque", type=float, default=0.10)
    p.add_argument("--home-contact-speed", type=float, default=0.20)
    p.add_argument("--home-contact-samples", type=int, default=3)
    p.add_argument("--home-timeout", type=float, default=180.0)
    # 开合循环
    p.add_argument("--cycle-feedforward", type=float, default=0.35)
    p.add_argument("--cycle-kp", type=float, default=5.0)
    p.add_argument("--cycle-kd", type=float, default=0.05)
    p.add_argument("--cycle-torque-limit", type=float, default=0.40)
    p.add_argument("--cycle-closing-torque-limit", type=float, default=None)
    p.add_argument("--cycle-tolerance", type=float, default=0.03)
    p.add_argument("--cycle-open-overshoot-limit", type=float, default=0.15)
    p.add_argument("--cycle-open-slow-distance", type=float, default=0.60)
    p.add_argument("--cycle-open-slow-speed", type=float, default=0.25)
    p.add_argument("--cycle-open-retract-margin", type=float, default=0.10)
    p.add_argument("--cycle-leg-timeout", type=float, default=30.0)
    p.add_argument("--contact-zero-tolerance", type=float, default=0.3)
    p.add_argument("--contact-force", type=float, default=0.5,
                   help="接触停止阈值 N")
    p.add_argument("--approach-rate", type=float, default=2.0)
    p.add_argument("--output-dir", type=Path,
                   default=Path(__file__).resolve().parent
                   / "hardware" / "experiment_results")
    # 触觉
    p.add_argument("--paxini-port", default="auto",
                   help="双PaXini串口；auto 按描述自动识别")
    p.add_argument("--left-slot", type=int, default=5)
    p.add_argument("--right-slot", type=int, default=8)
    p.add_argument("--paxini-force-limit", type=float, default=25.0,
                   help="闭合时任一侧法向合力安全阈值N")
    p.add_argument("--closing-velocity", type=float, default=0.01)
    p.add_argument("--closing-hold-time", type=float, default=1.0)
    p.add_argument("--require-contact", action="store_true",
                   help="闭合必须检测到接触，否则报错退出（原始接触测试语义）；"
                        "默认空载模式：走满闭合行程无接触也算正常完成")
    return p.parse_args(argv)


def run(args) -> int:
    motor_cfg = MotorConfig(
        backend=args.backend, channel=args.channel, interface=args.interface,
        motor_id=args.motor_id, master_id=args.master_id,
        pmax=args.pmax, vmax=args.vmax, tmax=args.tmax, rate=args.rate,
        lead_mm=args.lead_mm, stroke_m=args.stroke_mm / 1000,
        feedback_timeout=args.feedback_timeout,
        enable_timeout=args.enable_timeout,
        home_torque=args.home_torque, home_position_kp=args.home_position_kp,
        home_position_kd=args.home_position_kd,
        home_position_margin=args.home_position_margin,
        home_feedforward=args.home_feedforward,
        home_position_rate=args.home_position_rate,
        home_expected_open_position=args.home_expected_open_position,
        home_end_slow_distance=args.home_end_slow_distance,
        home_end_slow_rate=args.home_end_slow_rate,
        home_end_feedforward=args.home_end_feedforward,
        home_end_threshold_distance=args.home_end_threshold_distance,
        home_end_contact_torque=args.home_end_contact_torque,
        home_position_brake_torque=args.home_position_brake_torque,
        home_min_travel=args.home_min_travel,
        home_already_open_delay=args.home_already_open_delay,
        home_contact_torque=args.home_contact_torque,
        home_contact_speed=args.home_contact_speed,
        home_contact_samples=args.home_contact_samples,
        home_timeout=args.home_timeout,
        cycle_feedforward=args.cycle_feedforward, cycle_kp=args.cycle_kp,
        cycle_kd=args.cycle_kd, cycle_torque_limit=args.cycle_torque_limit,
        cycle_closing_torque_limit=args.cycle_closing_torque_limit,
        cycle_tolerance=args.cycle_tolerance,
        cycle_open_overshoot_limit=args.cycle_open_overshoot_limit,
        cycle_open_slow_distance=args.cycle_open_slow_distance,
        cycle_open_slow_speed=args.cycle_open_slow_speed,
        cycle_open_retract_margin=args.cycle_open_retract_margin,
        cycle_leg_timeout=args.cycle_leg_timeout,
        allow_no_contact=not args.require_contact,
        contact_zero_tolerance=args.contact_zero_tolerance,
        contact_force=args.contact_force, approach_rate=args.approach_rate,
        output_dir=args.output_dir)
    tactile_cfg = TactileConfig(
        port=args.paxini_port, left_slot=args.left_slot,
        right_slot=args.right_slot, force_limit=args.paxini_force_limit,
        closing_velocity=args.closing_velocity,
        closing_hold_time=args.closing_hold_time)
    try:
        validate_configs(motor_cfg, tactile_cfg)
    except ValueError as exc:
        print(f"参数无效：{exc}", file=sys.stderr)
        return 2

    model = GripperModel(motor_cfg.lead_mm / 1000, motor_cfg.stroke_m)
    state = SharedState(model)
    tactile = TactileMonitor(state, tactile_cfg)
    motor = MotorMonitor(state, motor_cfg, tactile_cfg)
    motor_started = False
    try:
        tactile.start()
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            snapshot = state.snapshot()
            if snapshot["safety"]["latched"]:
                raise RuntimeError(snapshot["safety"]["reason"])
            if snapshot["tactile"]["connected"]:
                break
            time.sleep(0.05)
        else:
            raise RuntimeError("等待 PaXini 上线超时，禁止电机找零")
        motor.start()
        motor_started = True
        while motor.is_alive():
            motor.join(timeout=0.1)
        snapshot = state.snapshot()
        if snapshot["gripper"]["error"] or snapshot["safety"]["latched"]:
            return 1
        return 0
    except KeyboardInterrupt:
        print("用户停止，正在请求零扭矩和失能。", flush=True)
        return 130
    finally:
        motor.stop()
        if motor_started:
            motor.join()  # Wait for motor cleanup before stopping tactile monitoring.
        tactile.stop()
        tactile.join()


if __name__ == "__main__":
    raise SystemExit(run(parse_args()))
