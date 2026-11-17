#!/usr/bin/env python3
# 流式读取原生 JSONL，提取 MAV_CMD_DO_SET_MODE=176 的本地发送、ACK 和状态确认。
# 飞控时间戳为微秒；ACK 与 wire 的差值仅是因果诊断，不能单独证明已切入 Offboard。
"""Stream ACK176 evidence without letting repeated SAFETY rows hide the ACK."""
import argparse
import json
from pathlib import Path


# 逐行输出有意义的事务变化，跳过损坏/非对象行；保留最终状态和发送/ACK 次数。
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    args = parser.parse_args()
    counts = {"command176": 0, "ack176": 0}
    # 最近一次命令发送时间及状态去重水位只用于本次日志扫描，不回写控制器事务。
    wire = None
    previous_phase = None
    previous_candidate = None
    last_state = None
    # 只摘取与 176 归属、候选 ACK、状态确认和时钟无归属有关的证据，避免 SAFETY 重复行淹没 ACK。
    fields = (
        "phase",
        "nav_state",
        "armed",
        "reason",
        "offboard_ack",
        "m1_176_ack_policy",
        "m1_176_ack_candidate",
        "m1_176_ack_accepted",
        "m1_176_ack_confirmed_by_state",
        "m1_176_ack_disposition",
        "m1_176_transaction_stage",
        "m1_176_local_generation",
        "m1_176_wire_stamp",
        "m1_176_causal_floor",
        "m1_176_ack_stamp",
        "m1_176_clock_unattributed_acks",
        "m1_176_verified_status_stamp",
    )
    with args.log.open(errors="replace") as stream:
        for line in stream:
            try:
                row = json.loads(line)
            except (ValueError, TypeError):
                continue
            if not isinstance(row, dict):
                continue
            kind = row.get("kind")
            # 先更新最近 wire，再解释后续 ACK；没有发送时间的 ACK 不计算假想时间差。
            if kind == "command_sink_audit" and row.get("command") == 176:
                counts["command176"] += 1
                wire = row.get("wire_timestamp")
                print("COMMAND176", json.dumps(row, ensure_ascii=False))
            elif kind == "ack" and row.get("command") == 176:
                counts["ack176"] += 1
                stamp = row.get("timestamp")
                if isinstance(wire, int) and isinstance(stamp, int):
                    row["ack_minus_wire_ms"] = (stamp - wire) / 1000
                print("ACK176", json.dumps(row, ensure_ascii=False))
            elif kind == "phase_event":
                before, after = row.get("phase_before"), row.get("phase_after")
                if before != after:
                    print("TRANSITION", json.dumps(row, ensure_ascii=False))
            if "m1_176_requested" in row or "offboard_ack" in row:
                last_state = {key: row[key] for key in fields if key in row}
                phase = row.get("phase")
                candidate = row.get("m1_176_ack_candidate")
                # 仅在阶段或 ACK 候选变化时打印状态，但始终更新 last_state 供末尾核查。
                if phase != previous_phase or candidate != previous_candidate:
                    print("STATE", json.dumps(last_state, ensure_ascii=False))
                    previous_phase, previous_candidate = phase, candidate
    print("COUNTS", json.dumps(counts))
    print("LAST_STATE", json.dumps(last_state, ensure_ascii=False))


if __name__ == "__main__":
    main()
