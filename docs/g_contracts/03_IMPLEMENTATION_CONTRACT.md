# 0.4.1 implementation contract

## Safety properties to enforce
1. Stage-one publication must be exact-zero-thrust at the FINAL publisher boundary.
2. Any non-zero attitude thrust must require an explicit allowed flight phase plus all required flight gates.
3. Any ARM command publication must require all four gates at the FINAL publisher boundary.
4. Fail closed. Diagnostics must explain why publication was blocked.
5. No independent heartbeat path may survive loss of supervised control health.
6. No new setpoint ownership conflict may be introduced.
7. No startup/update path may auto-enable or auto-resume flight.

## Design preference
Prefer a small, pure, testable policy/guard object that can be called from ROS adapter boundaries over scattering duplicated if-statements.
The guard should be unit-testable without ROS/DDS.

A reasonable shape (not mandatory) is:
- input: profile, phase, four gates, shutdown, operator lease, armed state, health, proposed thrust/command
- output: allow/deny + stable reason enum/string

Do not rewrite the overall state machine merely to satisfy this hardening task.

## Backward compatibility
Preserve stage-one `attitude-ready` behavior with gates false.
Preserve existing status consumers where possible; additive JSON fields are preferred.

## Socket/update hardening
If automatic stale-socket deletion cannot be made race-safe, do not implement it. Instead:
- detect stale-looking condition
- print exact diagnosis and safe manual command
- keep update aborted
This is acceptable and preferable to unsafe auto-cleanup.

## Version sensitive contract
Use PX4 v1.16 / px4_msgs release/1.16 semantics only.
Do not infer fields/constants from main.
