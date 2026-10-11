# G implementation ledger
Plan: docs/superpowers/plans/2026-10-04-safety-hardening.md
Spec: docs/g_contracts/01_G_TASK_SPEC.md
Frozen candidate ZIP hash matches supplied 091b7425...; all handoff hashes pass.
Baseline tests: five C++ programs pass; Python 8 tests OK with one AF_UNIX skip. Native C++ AF_UNIX check also skipped (EPERM).
Ruling: supplied formal implementation contract and explicit execution request authorize proceeding without another approval round.
Ruling: updater refuses any existing socket or active service without opening a controller session. Users must explicitly quiesce before activation; this intentionally narrows legacy live-stop update behavior and avoids diagnostic connections triggering control-session disconnect.
Ruling: advisory lifetime lock coordinates new cooperating starters/updaters; it does not authenticate same-UID malicious programs or legacy 0.4.0 actors.
Pre-flight: policy context shared between runtime and node; sink wrappers are the only attitude pair publication path; node supplies a freshly sampled context.

Implementation complete: Arm sink guard, stage-one exact-zero rejection, exhaustive phase allowlist, shared context, pair sinks, additive status, diagnostic-only socket refusal, lifetime lock and strict explicit activation.
RED evidence: G_RED_GREEN.txt; latest GREEN: G_TEST_OUTPUT.txt. Original 0.4.0 results: G_BASELINE_TEST_OUTPUT.txt.
Verification: 8 C++ programs exit 0 (native socket subcheck skip); Python 14 (12 pass / 2 skip); adapter fixture syntax, 12 shell and 9 Python AST pass. Humble/DDS/native transport UNKNOWN.
Invariant byte checks: G_INVARIANTS.json; 26 protected files; ack()/track() match 0.3.1/0.4.0/0.4.1. Input archive identity and original patch replay match.
Author whole-change review recorded in G_CODE_REVIEW.md. Fresh independent review could not run due quota; UNKNOWN, no independent approval claimed.
Release boundary: only the isolated copy was edited. G1 package creation/replay/checks precede persistence; optional G2 must be a separate addon after G1 delivery.
