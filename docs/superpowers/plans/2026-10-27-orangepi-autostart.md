# Orange Pi Autostart Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** One offline installer enables a boot-time, closed-gate native standby service with bounded logs and simple diagnostics.

**Architecture:** Separate standby supervisor reuses launcher ownership, config and socket primitives without changing the interactive flight path. systemd runs it as the installing user; an offline self-extractor carries the service manager and required Python modules.

**Tech Stack:** Bash, Python 3.10 stdlib + existing ROS Humble/PyYAML, systemd 249 (Ubuntu 22.04).

**Spec:** docs/superpowers/specs/2026-10-27-orangepi-autostart-design.md

## Global Constraints

- Eight ACTION_FLAGS false; only STATUS on the native console. No automatic Offboard, arm or takeoff.
- UART /dev/ttyS2 at 115200; ROS_DOMAIN_ID=0, ROS_LOCALHOST_ONLY=0; preserve original installation.
- Same operator lock, controller/Agent ownership checks; never stop a reused Agent or force-kill native.
- One 30-second dependency retry interval; controller log 4 MiB + two backups, supervisor 1 MiB + two backups; CSV disabled.
- System service with User/Home from passwd, noninteractive stdin, graceful stop without SIGKILL; no desktop/network dependency.

## Review Focus

- USB/FC power comes later: service stays waiting with an explicit reason, not failed silently or spawning duplicates.
- Manual flight entry already owns the lock: standby cannot create another controller or take ownership.
- A live native process survives the socket closing: request graceful exit and wait; do not restart beside it.
- Pi remains powered for hours: log/status storage remains bounded; retry attempts do not create unbounded sessions.
- Custom paths contain spaces, dollar signs or percent characters: generated unit preserves literal argv/environment.

### Task 1: Installable standby service

**Files:**
- Create: standby.py, standby.sh, autostart.py, autostart.sh, build_autostart_bundle.py, README_AUTOSTART_zh.md
- Test: tests/test_autostart.py, tests/test_standby.py
- Reuse unchanged: launcher.py, fc01_control.py, build_bundle.py integrity extractor

**Interfaces:**
- Consumes runtime_profile(base, False), preflight(root, agent, device), Channel, NodeStop, finish_agent, make_socket_path and private_dir.
- Produces standby.monitor(channel, node, stopped, observe), standby.supervise(root, agent, state_dir, stopped), autostart.render_unit(user, home, entry, project), autostart.main(argv), build_autostart_bundle.build(output, source=None).

- [ ] Write behavior tests: real socket/process standby lasting until stop, commands only STATUS despite input absence, graceful node exit, freshness timeout status, bounded actual log files; generated unit via systemd-analyze verify, extraction then execute fake service boundary and idempotent install/foreign-unit rejection.
- [ ] Run the new tests; expected FAIL because standby modules/installer are absent. Retain RED evidence.
- [ ] Implement standby supervisor, closed config and rotating logs. Implement service manager, safe unit quoting, boot shell and offline builder. Add minimal user instructions.
- [ ] Run `python3 -m unittest discover -s tests -v`; expected all tests PASS. Run Bash/Python 3.10 syntax and generated unit verification; expected exit 0. Build/extract actual self-extractor, verify package bytes.
- [ ] Commit the feature, record command/result in execution ledger. Perform one fresh-context whole-change review; fix substantive findings with RED→GREEN and full suite.
- [ ] Save the single verified installer to Library and deliver one install command plus expected status and scope.
