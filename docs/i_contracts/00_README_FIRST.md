# Agent I handoff — FC01 0.4.2 authority modes

You are Agent I, the implementation engineer for a NEW isolated release candidate:

`0.4.2-authority-modes`

Baseline is the actual reviewed and real-stage-one-tested `0.4.1-safety-hardening` source in `baseline/`.

## Evidence priority
1. Actual 0.4.1 source and reproducible tests.
2. `02_VERIFIED_FIELD_BASELINE.md` (new real-user evidence after G/H).
3. H independent review.
4. G delivery report.
5. PX4 v1.16 official docs/source for version-sensitive behavior.
6. Older F/D reports.
7. Inference; missing evidence => `UNKNOWN`.

## Absolute boundary
This is an OFFLINE implementation task. Do not connect FC01, do not start the real Agent/controller, do not change the user's `current`, do not write PX4 parameters, do not Arm, do not Takeoff, and do not output real non-zero setpoints.

The user has explicitly decided that BOTH operating concepts must remain possible:
- `RC_TAKEOVER`: computer/Orange Pi performs automatic Offboard takeoff/landing, but a real RC pilot can take over and emergency-stop.
- `AUTONOMOUS_NO_RC`: a deliberately selected autonomous mode that does not require RC, but requires its own independently confirmed PX4/autonomy failsafe contract.

Do not weaken 0.4.1 publisher-boundary hardening to achieve this.
