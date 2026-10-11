# Acceptance tests for 0.4.1-safety-hardening

All existing 0.4.0 tests must continue to pass.
All original ACK lifecycle regression tests must pass.

## Mandatory new safety tests
At minimum:

1. `stage_one_nonzero_thrust_rejected_at_final_boundary`
   - simulate valid stage-one phase and an upstream erroneous finite non-zero thrust
   - final publisher policy must deny/fault
   - no ROS wire message considered publishable

2. `stage_one_zero_thrust_allowed`
   - valid stage-one zero thrust remains publishable

3. `arm_blocked_if_flight_actions_false`
4. `arm_blocked_if_arm_enabled_false`
5. `arm_blocked_if_takeoff_enabled_false` if takeoff readiness is part of arm contract; if architecture intentionally separates this, document the contract and test the correct required set instead
6. `arm_blocked_if_parameters_confirmed_false`
7. `arm_blocked_wrong_phase`
8. `arm_blocked_on_shutdown`
9. `arm_blocked_without_live_operator_lease`
10. `arm_allowed_policy_only_when_all_required_inputs_true` — policy-level only, no real ROS/hardware

11. `nonzero_thrust_blocked_if_any_required_gate_false`
12. `nonzero_thrust_blocked_wrong_phase`
13. `nonzero_thrust_blocked_shutdown_or_unhealthy`

14. status/JSON contains `flight_parameters_confirmed` and all other flight gates

15. profile ownership regression: attitude_altitude still has no TrajectorySetpoint publisher
16. heartbeat supervision regression
17. command 400 sequential ACK/watermark regression

## Socket/update tests
18. regular file at socket path => refuse, never delete
19. symlink/unexpected path type => refuse, never delete
20. confirmed live listener => refuse update
21. stale-looking socket => either safe confirmed cleanup OR diagnostic-only failure according to chosen design
22. ambiguous/race/unknown => fail closed
23. install still never auto-switches current

## Packaging checks
- archive contains source/config/scripts/tools/tests/docs/README/VERSION/package.xml
- no build/install/log/cache/ELF artifacts
- patch applies cleanly to pristine 0.4.0
- replayed tree byte-matches delivered 0.4.1 source

## Do not claim
Passing these tests does NOT authorize:
- real Arm
- real non-zero thrust
- Takeoff
- physical flight
- Orange Pi production deployment
