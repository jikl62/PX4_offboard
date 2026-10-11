# G handoff — read this first

You are Agent G. Your job is **offline safety hardening**, not flight testing.

## Authority / evidence order
When materials conflict, use this order:

1. Current user's verified field/build evidence in `02_VERIFIED_BASELINE_AND_NEW_EVIDENCE.md`.
2. Actual candidate source in `candidate/` and baseline source in `baseline/`.
3. Agent D independent review.
4. PX4 v1.16 / px4_msgs release/1.16 official version-sensitive references.
5. Agent F delivery report.
6. A/B/C/E reports and manuals.
7. Inference. If evidence is missing, write `UNKNOWN`.

Do **not** substitute PX4 `main` for version-sensitive v1.16 behavior.

## Frozen facts
- `0.3.1-ackfix` is the known-good ground command/ACK baseline.
- `0.4.0-attitude-altitude` has been built successfully on the user's Ubuntu 22.04 + ROS2 Humble machine and was independently reviewed by D as `PASS WITH CONDITIONS` for props-off/disarmed smoke only.
- The user's `current` was explicitly switched to 0.4.0 after build/review; you must not touch it.
- Stage-one flight gates are all false.
- Arm/Takeoff remains forbidden.

## Your product
Create a **new, isolated** `0.4.1-safety-hardening` source candidate. Never modify 0.4.0 in place.

The primary target is to close D's Important finding I-01 (publisher-boundary defense in depth) and harden stale Unix-socket handling discovered during the 0.4.0 activation.

After G1 is complete, if time/context remains, proceed to the optional G2 fake-PX4 harness task in `07_OPTIONAL_G2_FAKE_PX4_HARNESS.md`.
