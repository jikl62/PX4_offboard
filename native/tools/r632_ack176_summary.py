#!/usr/bin/env python3
"""Stream ACK176 evidence without letting repeated SAFETY rows hide the ACK."""
import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    args = parser.parse_args()
    counts = {"command176": 0, "ack176": 0}
    wire = None
    previous_phase = None
    previous_candidate = None
    last_state = None
    fields = (
        "phase", "nav_state", "armed", "reason", "offboard_ack",
        "m1_176_ack_policy", "m1_176_ack_candidate", "m1_176_ack_accepted",
        "m1_176_ack_confirmed_by_state", "m1_176_ack_disposition",
        "m1_176_transaction_stage", "m1_176_local_generation",
        "m1_176_wire_stamp", "m1_176_causal_floor", "m1_176_ack_stamp",
        "m1_176_clock_unattributed_acks", "m1_176_verified_status_stamp",
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
                if phase != previous_phase or candidate != previous_candidate:
                    print("STATE", json.dumps(last_state, ensure_ascii=False))
                    previous_phase, previous_candidate = phase, candidate
    print("COUNTS", json.dumps(counts))
    print("LAST_STATE", json.dumps(last_state, ensure_ascii=False))


if __name__ == "__main__":
    main()
