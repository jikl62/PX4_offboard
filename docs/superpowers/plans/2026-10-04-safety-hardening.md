# 0.4.1 safety hardening implementation plan

> Execute inline under superpowers:executing-plans; final whole-change review.

**Goal:** Close D I-01 at the final Arm/attitude publishers and refuse unsafe stale-socket cleanup/activation.
**Architecture:** Pure BoundaryContext/Decision policy, guarded command/pair sink wrappers, fresh runtime context at ROS publish sites. Diagnostic-only existing socket refusal, a shared advisory socket lock for cooperating 0.4.1 starters/updaters, and checked explicit atomic activation.
**Tech:** C++17, Python standard library/POSIX, PX4 v1.16 pinned interfaces.
**Spec:** docs/g_contracts/03_IMPLEMENTATION_CONTRACT.md and 01_G_TASK_SPEC.md.

## Global constraints
- Four gates false in delivered YAML; package=0.4.1, VERSION=0.4.1-safety-hardening.
- Never touch frozen inputs, real current, devices, parameters, Agent or real ROS chain.
- Preserve ACK ack()/track(), command/target 1/191, watermark and five-second window.
- No automatic socket deletion; UNKNOWN/race/type/ownership => refusal.
- Stage-one PRESTREAM/REQUEST_OFFBOARD/OFFBOARD_READY zero-thrust remains available with gates false.

## Review focus
- Denormal and lateral/positive thrust must be denied without float-rounding hiding nonzero.
- RequestArm zero-output may remain armed while ACK is pending; an Arm send always requires disarmed.
- Shutdown cannot emit nonzero thrust; it releases control to PX4/operator at final boundary.
- One-shot stage-one session has no interactive marker; it remains eligible while connected.
- Lock identity/path replacement and unknown service state must abort activation.

### Task 1: Final publication guards and diagnostics
Files: new publication_guard.hpp/publication_boundary.hpp; runtime.hpp and node integration; tests/test_publication_boundary.cpp.
Interfaces: BoundaryContext captures Config/profile/phase/feedback/shutdown/control/session/reference facts; Decision has allow plus stable reason; publish_arm_at_boundary and publish_attitude_at_boundary call sinks only when eligible.
- [x] Write tests: stage-one all components/tiny nonzero rejected, exact zero allowed, four-gate Arm truth table plus phase/shutdown/lease/health, nonzero flight restrictions, ownership and supervised pair counts.
- [x] Run RED against missing guard (assert feature absent).
- [x] Implement pure policy and sink wrappers; expose read-only lease/context queries, integrate unconditional Arm gate into publish_command and paired guard into node.
- [x] Add additive gate/heartbeat/output/block reason JSON; test real JSON fragment and adapter linkage.
- [x] Run GREEN complete scripts/test.sh, preserving ACK/flight lifecycle tests.

### Task 2: Socket diagnostics and race-safe cooperating activation
Files: unix_server.hpp, tools/socket_safety.py/tools/update.py, scripts/update.sh, tests/test_socket_safety.py and test_socket.cpp.
Interfaces: lifetime C++/Python lock at socket_path+'.lock'; existing socket/type => refuse with no deletion; activation under lease checks path and service before os.replace.
- [x] Write RED: regular/symlink/FIFO/stale-looking paths retained, live/lock busy, path race and unknown service fail, current unchanged.
- [x] Implement strict owned file/dir/lock validation and diagnostic-only socket handling; no connect/delete in updater.
- [x] Run GREEN fixtures; report native AF_UNIX limitations honestly.
- [x] Preserve install script byte identity; explicit update/rollback fixture passes with absent socket/inactive service.

### Task 3: Release verification and complete G1 delivery
Files: VERSION/package.xml/version-bound tests/README, docs/G_* and six primary output artifacts.
- [x] Preserve existing tests with only version/lock-fixture adaptations; run complete suite, syntax and adapter fixture compilation.
- [x] Review boundary coverage and invariant byte hashes; fix any demonstrated defects with RED/GREEN.
- [x] Create clean archives and full unified diff, replay onto pristine 0.4.0, compare every file byte/mode, checksum outputs.
- [ ] Final external step: save the six verified G1 outputs before optional G2. Real ROS/Humble/DDS/flight are explicitly UNKNOWN.
