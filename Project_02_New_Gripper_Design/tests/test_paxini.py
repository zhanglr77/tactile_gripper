# -*- coding: utf-8 -*-
"""PaXini 触觉传感器硬件集成测试 — 检查读数与归零功能。

需要连接真实的 PaXini GEN3 传感器才能运行。
用法：
    python3 tests/test_paxini.py
"""
from __future__ import annotations

import sys
import os
import time
import traceback

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from hardware.paxini import PaxiniSensor, ProtocolError

FORCE_ZERO_TOLERANCE = 0.5  # 归零后允许的最大残余力 (N)
READ_TIMEOUT = 2.0
NUM_FRAMES_TO_READ = 5


def run_tests():
    passed = 0
    failed = 0
    errors = []

    def check(name, fn):
        nonlocal passed, failed
        try:
            fn()
            print(f"  [PASS] {name}")
            passed += 1
        except AssertionError as e:
            print(f"  [FAIL] {name}: {e}")
            failed += 1
            errors.append((name, str(e)))
        except Exception as e:
            print(f"  [ERROR] {name}: {e}")
            failed += 1
            errors.append((name, traceback.format_exc()))

    # ---- 连接传感器 ----
    print("连接 PaXini 传感器...")
    try:
        sensor = PaxiniSensor(port="auto")
    except (RuntimeError, ProtocolError) as exc:
        print(f"无法连接传感器: {exc}")
        print("请检查传感器是否已接入并上电。")
        sys.exit(1)

    print(f"已连接: {sensor}\n")

    # ---- 读数测试 ----
    print("=== 读数测试 ===")

    def test_has_connected_slots():
        assert len(sensor.slots) > 0, "没有检测到任何已连接的 slot"

    def test_version_not_empty():
        assert sensor.version, "固件版本号为空"

    def test_can_read_frame():
        frame = sensor.read(timeout=READ_TIMEOUT)
        assert frame is not None, "读取返回 None"
        assert len(frame.slots) > 0, "读取到的帧中没有 slot 数据"
        for slot in frame.slots:
            print(f"    {slot.name}: Fx={slot.fx_n:.2f} Fy={slot.fy_n:.2f} Fz={slot.fz_n:.2f} N")

    def test_multiple_frames():
        frames = []
        for _ in range(NUM_FRAMES_TO_READ):
            frames.append(sensor.read(timeout=READ_TIMEOUT))
        assert len(frames) == NUM_FRAMES_TO_READ, (
            f"只读取到 {len(frames)}/{NUM_FRAMES_TO_READ} 帧")
        for f in frames:
            assert len(f.slots) == len(sensor.slots), (
                f"帧 slot 数不一致: 期望{len(sensor.slots)}, 实际{len(f.slots)}")

    def test_force_values_in_range():
        frame = sensor.read(timeout=READ_TIMEOUT)
        for slot in frame.slots:
            assert -25.6 <= slot.fx_n <= 25.6, f"{slot.name} Fx 超出范围: {slot.fx_n}"
            assert -25.6 <= slot.fy_n <= 25.6, f"{slot.name} Fy 超出范围: {slot.fy_n}"
            assert 0.0 <= slot.fz_n <= 25.6, f"{slot.name} Fz 超出范围: {slot.fz_n}"

    def test_read_normal_forces():
        left_fz, right_fz = sensor.read_normal_forces(timeout=READ_TIMEOUT)
        assert isinstance(left_fz, float), f"left_fz 类型错误: {type(left_fz)}"
        assert isinstance(right_fz, float), f"right_fz 类型错误: {type(right_fz)}"
        print(f"    左={left_fz:.2f} N, 右={right_fz:.2f} N")

    check("有已连接的 slot", test_has_connected_slots)
    check("固件版本号非空", test_version_not_empty)
    check("读取单帧数据", test_can_read_frame)
    check("连续读取多帧", test_multiple_frames)
    check("力值在合理范围内", test_force_values_in_range)
    check("read_normal_forces 返回左右法向力", test_read_normal_forces)

    # ---- 归零测试 ----
    print("\n=== 归零测试 ===")

    def test_calibrate_succeeds():
        sensor.calibrate()

    def test_readings_near_zero():
        sensor.calibrate()
        time.sleep(0.3)
        frame = None
        for _ in range(3):
            frame = sensor.read(timeout=READ_TIMEOUT)
        for slot in frame.slots:
            assert abs(slot.fx_n) < FORCE_ZERO_TOLERANCE, (
                f"{slot.name} Fx 归零后残余力过大: {slot.fx_n:.2f} N")
            assert abs(slot.fy_n) < FORCE_ZERO_TOLERANCE, (
                f"{slot.name} Fy 归零后残余力过大: {slot.fy_n:.2f} N")
            assert slot.fz_n < FORCE_ZERO_TOLERANCE, (
                f"{slot.name} Fz 归零后残余力过大: {slot.fz_n:.2f} N")
        print("    归零后各 slot 力值均在容差范围内")

    check("calibrate() 执行成功", test_calibrate_succeeds)
    check("归零后读数接近零", test_readings_near_zero)

    # ---- 清理 ----
    sensor.close()

    # ---- 汇总 ----
    print(f"\n{'='*40}")
    print(f"总计: {passed + failed} | 通过: {passed} | 失败: {failed}")
    if errors:
        print("\n失败详情:")
        for name, msg in errors:
            print(f"  - {name}: {msg}")
    print()
    sys.exit(0 if failed == 0 else 1)


if __name__ == "__main__":
    run_tests()
