#!/usr/bin/env python3
# S10 只读时间线工具：从 JSONL/CSV 提取状态与心跳变化，不解析 ULog、不开 ROS、不联系 FC01。
# 时间列缺失时用序号/行号展示顺序，不能把该显示值当作物理秒或飞控测量时间。
"""Read-only S10 controller timeline from JSONL state messages or CSV.

Does not parse ULog, connect ROS/DDS, arm, or communicate with FC01.
For PX4 ULog physics use Agent A's validated ULog extractor and contract.
"""
import argparse
import csv
import json
import sys
from pathlib import Path


# CSV 按字段读取；JSONL 跳过损坏行，只接受状态类对象并保留原行号。
def records(path):
    with path.open(encoding="utf-8", errors="replace") as stream:
        if path.suffix.lower() == ".csv":
            yield from csv.DictReader(stream)
        else:
            for idx, line in enumerate(stream, 1):
                try:
                    v = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if isinstance(v, dict) and v.get("kind") in ("state", "status", None):
                    v["_line"] = idx
                    yield v


# 兼容 CSV 文本与 JSON bool；这只是历史日志解释，不用于生产动作授权。
def truth(value):
    if isinstance(value, bool):
        return value
    return str(value).lower() in ("true", "1", "yes")


# 保留阶段/心跳变化和爬升下降超时，并统计已解锁 Offboard 下心跳关闭的观测。
def summarize(path):
    events = []
    last_phase = None
    last_hb = None
    armed_offboard_loss = []
    n = 0
    for r in records(path):
        phase = r.get("phase", "")
        if not phase:
            continue
        n += 1
        time = r.get("steady_s", r.get("seq", r.get("_line", n)))
        phase_change = phase != last_phase
        # 兼容早期日志字段；缺失心跳不假装已开启，也不推断飞控实际采用目标。
        hb = r.get("heartbeat_enabled", r.get("attitude_output_enabled"))
        if phase_change:
            events.append(
                (time, "PHASE", f"{last_phase or 'START'} -> {phase}", r.get("reason", ""))
            )
        if hb is not None and truth(hb) != last_hb:
            events.append((time, "HEARTBEAT", "ON" if truth(hb) else "OFF", ""))
        if "DESCENT_TIMEOUT" in str(r.get("reason", "")) or "LIFTOFF_TIMEOUT" in str(
            r.get("reason", "")
        ):
            if not events or events[-1][3] != r.get("reason", ""):
                events.append((time, "TIMEOUT", phase, r.get("reason", "")))
        armed = truth(r.get("armed", r.get("px4_armed", False)))
        offboard = truth(r.get("offboard", r.get("px4_offboard", False)))
        # 只记录未取证交权窗口，不把本地观察自动升级为真实事故或 PX4 回退成功。
        if armed and offboard and hb is not None and not truth(hb):
            armed_offboard_loss.append((time, phase))
        last_phase = phase
        if hb is not None:
            last_hb = truth(hb)
    print(f"INPUT: {path.name}; parsed_state_rows={n}")
    for t, kind, name, reason in events:
        print(f"{t}: {kind}: {name}" + (f" | {reason}" if reason else ""))
    print(f"ARMED_OFFBOARD_HEARTBEAT_OFF_OBSERVATIONS={len(armed_offboard_loss)}")
    if armed_offboard_loss:
        print(f"FIRST_UNVERIFIED_HANDOFF={armed_offboard_loss[0]}")
    # 没有可解析状态时明确返回 2，防止空日志显示“无故障”而误判通过。
    if n == 0:
        print(
            "NO_PARSED_STATE_ROWS: provide runtime JSONL or controller CSV; ULog is separate",
            file=sys.stderr,
        )
        return 2
    return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path)
    args = parser.parse_args()
    sys.exit(summarize(args.path))
