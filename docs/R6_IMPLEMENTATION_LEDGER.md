> 历史 R6 基线记录，保留原结论与来源。本文件不是 R6.1 的当前结论；请先读根目录 `README_FIRST.md` 和 `docs/R6_1_SAFETY_EVIDENCE.md`。

# R6 ledger — plan: docs/superpowers/plans/2026-10-09-r6-control-recovery.md

Baseline: R5 full source from verified uploaded handoff, independent directory copy.
Intent: immediate offline implementation as explicitly requested; no additional approval flow.
Interfaces: Core phases → boundary recovery phases → Runtime21 → Node final sinks.
Known external gap: actual FC01 fallback flight task and native Humble build unavailable here.
Historical B timeout test asserts the defect's former terminal behavior. Preserve its exact R5
bytes separately and replace only that obsolete expectation in the active R6 version;
all original C probes stay byte-identical and are replayed independently.

Task 1: complete offline implementation; timeout RED on R5 -> GREEN within bounded recovery on R6.
Task 2: complete local sink audit/phase events; actual Node fixture syntax PASS, native Humble UNKNOWN.
Task 3: configurations/check-only script/source packaging in progress.
Final review: one fresh read-only reviewer; 2 Important findings reproduced and fixed.
Final fixed: terminal ControlledDescent/Disarm guarded labels -> UNKNOWN when output is stopped (RED/GREEN).
Final fixed: dual silent tail and sink exception cannot pass event audit (RED/GREEN).
Final fixed: partial target commit retains attempted and committed thrust evidence (RED/GREEN).
Additional fix: external Safety ground terminal uses contiguous new land dwell rather than two gapped endpoints (RED/GREEN).
Final suite: scripts/test.sh exit 0; 21 C++ executables; Python 28 pass / 2 skipped of 30.
Minor deferred: synthetic same-stamp mode rewrite at pure Core API can reach generic mode-exit description;
production Stamp.accept rejects same-stamp mutation. C must inspect all real ingress paths.
Extended ownership gate: RED at 38.05s; powered flight P0 remains BLOCKED. No flight release assertion.
