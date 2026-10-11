# Verified baseline and new evidence for G

## Known-good 0.3.1 ground transaction
Verified on real FC01:
- mode command 176 executes
- VehicleStatus changed to Stabilized
- ACK was `command=176 result=0 target_system=1 target_component=191 from_external=false`
- ground transaction reached PASS

## 0.4.0 candidate identity
Candidate source ZIP reviewed by D:
`091b7425f8ba0a421545124965e026a7f2df62f5ba5dab54d4fec8d34c0a6bc8`

User later verified the local ZIP had exactly the same SHA256.

px4_msgs commit used in actual build:
`392e831c1f659429ca83902e66820d7094591410`

## Actual Ubuntu 22.04 + ROS2 Humble build
The user executed `bash scripts/test.sh` in the extracted 0.4.0 source and got PASS for:
- ACK lifecycle
- attitude-altitude runtime
- attitude/vertical/ownership
- PX4 attitude wire/profile plan
- socket regular-file protection / stamp regression
- 8 Python tests: OK

Then the user executed:
`bash scripts/install.sh ~/fc01_system/releases/0.3.1-ackfix-20261004T090116Z-Pdwbao/workspace/src/px4_msgs`

Actual build result:
- `px4_msgs` finished in ~2m13s
- `jikl_fc01_system` finished in ~29s
- `Summary: 2 packages finished`
- built release: `/home/jikl62/fc01_system/releases/0.4.0-attitude-altitude-20261004T134801Z-Jm0NbG`
- install did NOT auto-switch current

Post-build readback:
- VERSION = `0.4.0-attitude-altitude`
- `flight_actions_enabled: false`
- `arm_enabled: false`
- `takeoff_enabled: false`
- `flight_parameters_confirmed: false`

The candidate ZIP SHA matched D's reviewed ZIP exactly, and the built workspace px4_msgs commit matched the pinned commit above.

## D independent review
Verdict: `PASS WITH CONDITIONS`.
No confirmed Critical finding.
Allowed scope: props-off, disarmed stage-one smoke only.
Arm/Takeoff not authorized.

Important finding I-01 remains open for 0.4.0:
- four-gate final publisher-boundary defense is incomplete
- stage-one source reachability is still zero-thrust/no-Arm by current control flow
- this finding blocks future stage-two authorization, not the limited stage-one smoke

I-02 (build evidence gap) is considered CLOSED by later user build/hash/gate evidence above.

## Stale Unix socket event
During activation of 0.4.0:
- `pgrep -af 'fc01_system_node|MicroXRCEAgent'` returned nothing
- user service was `inactive`
- `~/.cache/fc01/control.sock` existed as `srw-------`
- `ss -xlpn` showed no listener
- update attempt returned `[Errno 111] Connection refused` and safely left current at 0.3.1
- human removed only the confirmed stale socket
- subsequent update succeeded

Current was then explicitly switched to 0.4.0 and safety gates remained false.

## Important workflow constraint
0.4.0 remains the frozen version intended for the next real stage-one smoke when hardware is available.
G must not replace or modify it. G produces 0.4.1 as a separate future candidate.
