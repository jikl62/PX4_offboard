# Mandatory acceptance tests for 0.4.2-authority-modes

Run ALL 0.4.1 tests unchanged or with only necessary version/diagnostic adaptations.

Add deterministic tests covering at minimum:

1. default authority is UNSET
2. UNSET cannot Arm at state-machine admission
3. UNSET cannot Arm at final publisher boundary
4. UNSET cannot publish non-zero thrust at final boundary
5. authority selection accepted only from verified safe IDLE/disarmed/landed/at_rest/output-off state
6. authority change rejected once preparation is locked
7. authority cannot switch during REQUEST_ARM/ARMED_GROUND/THRUST_RAMP/ASCEND/HOLD/DESCENT/LANDED/REQUEST_DISARM
8. safe ground completion unlocks selection only after outputs stopped and no pending transaction

RC_TAKEOVER:
9. no/freshness-failed RC => Arm denied
10. takeover confirmation false => Arm/non-zero denied
11. fully satisfied RC policy can reach policy-level Arm allow (synthetic only)
12. nav_state leaves Offboard / explicit operator takeover => latch takeover, stop/release outputs, never auto re-request Offboard
13. RC loss during an active RC-policy attempt follows documented fail-closed/release behavior

AUTONOMOUS_NO_RC:
14. no RC is not itself a blocker
15. autonomous_failsafe_ready=false => Arm denied at final boundary
16. autonomous_failsafe_ready=false => non-zero thrust denied at final boundary
17. synthetic complete autonomous policy can reach policy-level allow without setting RC-ready true
18. autonomous mode never changes PX4 parameters

Regression/security:
19. stage-one exact zero thrust behavior unchanged
20. stage-one injected non-zero still denied
21. all four flight gates still independently required for Arm/non-zero
22. ACK lifecycle / command 400 sequencing unchanged
23. socket/update hardening unchanged
24. install never auto-switches current
25. status JSON exposes authority mode/lock/readiness and stable reason
26. G2/fake-PX4 harness is extended or supplemented with authority/takeover scenarios where practical

PASS is offline evidence only. No test in this task authorizes real Arm/Takeoff.
