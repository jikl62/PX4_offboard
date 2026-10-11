# Implementation ledger — plan: docs/superpowers/plans/2026-10-05-authority-modes.md

Supplied baseline commit: 770e22beb4113e25ffbce3fb00f397517d86b88d.
Input handoff SHA256: 17/17 verified. Actual baseline source and reports D/F/G/G2/H read. Source extracted with original file modes and byte comparison; baseline code remains pristine.

Baseline tests: 8 C++ programs PASS; adapter fixture PASS; Python 14 run / 12 PASS / 2 native AF_UNIX SKIP (UNKNOWN). Evidence outside candidate at evidence/BASELINE_TEST_OUTPUT.txt.

Pre-flight interfaces: Task 1 policy State/Inputs feed Task 2 Core/Runtime and Task 3 node; all final boundaries consume the same explicit state. Task 2 Snapshot new facts are produced by Task 3 adapter. Task 4 compares against original supplied ZIP, not git metadata.

Ruling: Use the supplied formal contract as the approved design and execute the explicitly requested offline implementation without another approval cycle. Work is a new isolated release copy, with a local git baseline for review; there is no external repository to merge or push.

Ruling: Keep UNSET stage-one usable and lock already at SELF_CHECK. Require explicit safe ground reset after completion/takeover; general and publication faults require restart. Cost: conservative operational refusal, never automatic reacquisition.

Ruling: Native AF_UNIX denied in this environment stays UNKNOWN; do not promote fixtures or H's different environment to native PASS.

Task 1: complete. Boundary RED exit134 (old complete Arm context allowed without authority); GREEN strict full suite exit0, 9 C++ programs + adapter fixture + 12/14 Python PASS, 2 native socket UNKNOWN. Policy includes both false-by-default confirmations, immutable attempt identity and re-evaluation at both sinks. One test syntax error (template comma in assert macro) was corrected before GREEN; not product RED.

Task 2: complete. Upstream RED exit134 with all old predicates satisfied (UNSET takeoff was accepted). Recovery RED exit134 (new prepare after disconnect without ground reset). GREEN full suite exit0: 10 C++ programs, adapter fixture, 12/14 Python PASS and 2 native socket UNKNOWN. All eight flight phases reject authority switching; safe completion requires output-off/no-pending explicit reset. ACK matcher/track untouched. Generic faults cannot be cleared through LAND/STOP.

Task 3: complete. Console syntax/one-shot RED showed missing commands; RC helper RED showed absent distinct observation. GREEN full suite exit0: 11 C++ tests + adapter fixture; Python 18 run / 16 PASS / 2 native socket UNKNOWN. New real-core harness 24/24, repeat JSON equal; original frozen G2 separately 5/5 and 39/39 baseline-only. Added regression for Kill retention: unmapped/duplicate cannot clear known Kill (RED exit134, then GREEN). Diagnostic BATTERY_PREFLIGHT_REQUIRED now separates battery/preflight from RC authority reasons (RED/GREEN). VERSION/package changed, old export/install tests adapted only expected release names.

Task 4 review: one fresh read-only whole-change reviewer inspected baseline 770e22b to frozen head 966a18c. Initial verdict BLOCK (offline source only), 3 Important + 1 Minor. Independently reproduced all four with assert-abort regressions; the first draft of the regression had a wire-template compile error and an inactive TAKEOVER assertion, corrected before the product RED evidence. Correct REDs are in I_REVIEW_REGRESSIONS_RED.txt. Single fix pass: require two advancing raw RC/switch samples within the original flags_timeout window (also after stale gaps); independent monotonic restart-required fault state; observe switch_changes including lost intermediate transitions; immediate takeover facts veto diagnostics and takeoff admission. Targeted regressions GREEN; final full suite exit0: 12 C++ programs + actual adapter fixture, Python 18 run / 16 PASS / 2 native transport SKIP. No second review was requested; final validation is implementer evidence, not a new independent-review verdict.

Task 4 invariants: 17/17 handoff hashes, pristine baseline and G2 bytes/modes, 14 G2 frozen headers equal baseline, 38 protected candidate files byte/mode equal, ACK and track byte equal, original Config/default validation unchanged after removing two booleans, all existing YAML values and 21 numerical sentinels unchanged; four flight gates and both confirmations false. See I_INVARIANTS.json.

Ruling: The handoff already chooses isolated source candidate + patch + six release artifacts. Retain the local implementation branch for provenance and package the requested result; no external merge, push, current switch or redundant integration menu applies.

Task 4 package verification: pristine supplied baseline replay succeeded with --fuzz=0 and no offsets; complete file/directory paths, bytes and modes equal the candidate. ZIP CRC passed, ZIP/TAR extraction trees equal candidate and each other, no ELF/cache/build/install/log/.git found, and external report equals embedded report. 132 source files, 16 subdirectories. After this final documentation closure the archive/patch verification is regenerated against the exact committed source; external PACKAGE_VERIFICATION/SHA256SUMS contain the final machine results. All six deliverables are prepared for the final save operation.
