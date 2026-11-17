#!/usr/bin/env python3
# 把同程 ULog 与控制器 JSONL 对齐后，驱动当前 C++ 垂直反馈监测器。
# z/vz/z_deriv 使用 PX4 NED，单位 m、m/s，向下为正；回放只证明历史输入下的检测，不能预测改控制后的真实轨迹。
"""Replay recorded vertical feedback through the production C++ guard.

This tests detection on historical inputs, not the aircraft's counterfactual
motion after a changed controller output. No ROS, device or network access.
Requires numpy and pyulog; the ordinary offline checks do not require pyulog.
"""
import argparse
import csv
import hashlib
import io
import json
from pathlib import Path
import subprocess
import tempfile

import numpy as np
from pyulog import ULog

ROOT = Path(__file__).resolve().parents[1]
# 明确列出同程日志对；文件名仅用于选样，真正时钟对应还要用 timesync 与记录数值交叉验证。
PAIRS = [
    ("log_0_2026-10-18-15-38-22.ulg", "ramp_guard_20261018_153730.log"),
    ("log_1_2026-10-18-15-40-08.ulg", "ramp_guard_20261018_153942.log"),
    ("log_2_2026-10-18-15-40-56.ulg", "ramp_guard_20261018_154027.log"),
    ("log_3_2026-10-18-15-43-28.ulg", "ramp_guard_20261018_154252.log"),
    ("log_4_2026-10-18-15-44-46.ulg", "ramp_guard_20261018_154407.log"),
]


# 提取带 local_stamp 的 phase_event，跳过普通文本；没有原始样本时间的行不构成回放输入。
def events(path):
    result = []
    for line in path.read_text().splitlines():
        try:
            event = json.loads(line)
        except ValueError:
            continue
        if event.get("kind") == "phase_event" and event.get("local_stamp"):
            result.append(event)
    return result


# 读取定位和时钟数据，选择斜坡到原终止的样本，验证对齐误差后送入独立 C++ 程序。
def replay(binary, ulg_path, controller_path, output):
    ulog = ULog(str(ulg_path))
    local = ulog.get_dataset("vehicle_local_position").data
    sync = ulog.get_dataset("timesync_status").data
    history = events(controller_path)
    ramp = next(e for e in history if e["phase_after"] == "THRUST_RAMP")
    terminal = next(e for e in history if e["phase_after"] == "SAFETY")

    # PX4 timesync offset maps the companion's translated message time back
    # into the boot clock. This mapping is for offline alignment only.
    # 利用 PX4 estimated_offset 将伴随侧时间映射到飞控 boot 微秒；只用于离线对齐。
    def boot_time(host):
        return host + np.interp(
            host, sync["remote_timestamp"].astype(float), sync["estimated_offset"].astype(float)
        )

    start, end = boot_time(ramp["local_stamp"]), boot_time(terminal["local_stamp"])
    selected = np.flatnonzero((local["timestamp"] >= start) & (local["timestamp"] <= end))
    # 不足三个样本无法建立有意义动态观测，直接拒绝而不靠插值补出不存在的真实测量。
    if len(selected) < 3:
        raise RuntimeError(f"insufficient aligned samples: {ulg_path.name}")
    # Verify the clock mapping against controller-recorded z/vz; do not infer
    # alignment solely from the human-readable filename or ULog start time.
    errors = []
    flight_history = [
        e for e in history if ramp["monotonic_s"] <= e["monotonic_s"] <= terminal["monotonic_s"]
    ]
    times = local["timestamp"].astype(float)
    for e in flight_history:
        target = boot_time(e["local_stamp"])
        j = int(np.clip(np.searchsorted(times, target), 1, len(times) - 1))
        j = min((j - 1, j), key=lambda k: abs(times[k] - target))
        # ULog local-position is downsampled relative to DDS (notably in the
        # short 15:43 transient). Bound velocity alignment error by the
        # neighbouring recorded velocity change, not a fixed low-speed
        # threshold that treats acceleration between log samples as a bad clock.
        nearby = local["vz"][max(0, j - 2) : min(len(times), j + 3)].astype(float)
        # ULog 与 DDS 采样频率不同，允许量结合邻近真实速度变化，避免把加速度误当时钟错位。
        velocity_allowance = 0.025 + max(0.025, float(np.max(np.abs(np.diff(nearby)))))
        if abs(float(local["vz"][j]) - e["vz"]) > velocity_allowance:
            raise RuntimeError(
                f"velocity alignment exceeds local sample variation: {ulg_path.name}"
            )
        errors.append(
            (
                abs(times[j] - target) * 1e-6,
                abs(float(local["z"][j]) - e["z"]),
                abs(float(local["vz"][j]) - e["vz"]),
            )
        )
    max_errors = np.max(errors, axis=0)
    if np.any(max_errors[:2] > np.array([0.060, 0.080])):
        raise RuntimeError(f"clock/value alignment failed: {ulg_path.name}: {max_errors}")
    # 向 C++ 输入源采样时间与 NED 三项原始反馈，保持原顺序；不是发送 PX4 控制指令。
    stream = "".join(
        f"{int(local['timestamp_sample'][i])} {float(local['z'][i]):.12g} "
        f"{float(local['vz'][i]):.12g} {float(local['z_deriv'][i]):.12g}\n"
        for i in selected
    )
    run = subprocess.run([str(binary)], input=stream, text=True, capture_output=True, check=True)
    rows = list(csv.reader(io.StringIO(run.stdout)))
    if not rows:
        raise RuntimeError("empty C++ replay output")
    (output / (ulg_path.stem + "_guard.csv")).write_text(
        "timestamp_sample,healthy,suspect,rate_error_mps,position_rate_error_mps,disagreement_s,reason\n"
        + run.stdout
    )
    last = rows[-1]
    fault = last[1] == "0"
    trigger = int(last[0])
    before = [e for e in flight_history if boot_time(e["local_stamp"]) <= trigger]
    historical = before[-1] if before else ramp
    # 摘要同时给原日志 SHA、对齐误差和原终止原因，保留证据来源及反事实限制。
    return {
        "ulog": ulg_path.name,
        "ulog_sha256": hashlib.sha256(ulg_path.read_bytes()).hexdigest(),
        "controller_log": controller_path.name,
        "controller_sha256": hashlib.sha256(controller_path.read_bytes()).hexdigest(),
        "aligned_samples": len(selected),
        "evaluated_samples": len(rows),
        "max_alignment_time_error_s": round(float(max_errors[0]), 6),
        "max_alignment_z_error_m": round(float(max_errors[1]), 6),
        "max_alignment_vz_error_mps": round(float(max_errors[2]), 6),
        "guard_triggered": fault,
        "guard_reason": last[6],
        "guard_time_from_ulog_start_s": (
            round((trigger - ulog.start_timestamp) * 1e-6, 6) if fault else None
        ),
        "historical_requested_thrust_near_trigger": (
            abs(historical["thrust_planned_z"])
            if fault and historical.get("thrust_planned_z") is not None
            else None
        ),
        "historical_phase_near_trigger": historical["phase_after"] if fault else None,
        "original_terminal_reason": terminal["reason"],
        "interpretation": "Historical-input detection only; no claim of improved real hover or physical thrust.",
    }


# 临时构建监测回放程序，输出逐样本 CSV 和机器可读摘要；需要 numpy/pyulog，但不运行 ROS。
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ulog-dir", type=Path, required=True)
    parser.add_argument("--controller-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="fc01_feedback_replay_") as tmp:
        binary = Path(tmp) / "replay"
        subprocess.run(
            [
                "g++",
                "-std=c++17",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-pedantic",
                "-I",
                str(ROOT / "src/jikl_fc01_system/include"),
                str(ROOT / "tests/vertical_feedback_replay.cpp"),
                "-o",
                str(binary),
            ],
            check=True,
        )
        results = [
            replay(binary, args.ulog_dir / ulg, args.controller_dir / log, args.output_dir)
            for ulg, log in PAIRS
        ]
    (args.output_dir / "replay_summary.json").write_text(
        json.dumps(results, indent=2, ensure_ascii=False) + "\n"
    )
    for row in results:
        print(
            row["ulog"],
            row["guard_time_from_ulog_start_s"],
            row["guard_reason"] or "NO_NEW_GUARD_BEFORE_ORIGINAL_EXIT",
        )


if __name__ == "__main__":
    main()
