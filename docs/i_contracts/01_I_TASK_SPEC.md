# Formal Task — Agent I: implement 0.4.2-authority-modes

## Goal
Create a new immutable source candidate derived from `0.4.1-safety-hardening` that introduces an explicit companion-side authority policy selectable BEFORE automatic flight:

- `UNSET`
- `RC_TAKEOVER`
- `AUTONOMOUS_NO_RC`

The user wants one software stack for PC now and Orange Pi later. The authority choice is a companion-controller safety policy above PX4 flight modes, not a replacement for PX4's internal modes/failsafes.

## 1. Selection and lock contract
Default after process start must be `UNSET`.

Add live-console/operator API conceptually equivalent to:
- `authority status`
- `authority rc`
- `authority autonomous`

Exact syntax may be adapted to the existing protocol, but must be explicit and testable.

Changing authority mode is allowed ONLY when all are true:
- phase is IDLE
- `armed=false`
- `landed=true`
- `at_rest=true`
- no attitude target publication active
- no Offboard heartbeat active
- no flight started / no pending Arm/Disarm transaction
- no latched publication/safety fault requiring restart/reset

Once preparation begins (no later than PRESTREAM), authority becomes locked. It stays locked throughout Offboard preparation, Arm, flight, descent, disarm, and recovery. It may become selectable again only after a verified safe return to IDLE/disarmed/landed/at_rest with outputs stopped.

Piped/non-live single commands must not silently change authority policy. Preserve the live operator/session model unless you provide a stricter equivalent.

## 2. Do NOT reuse one ambiguous `manual_ready` boolean as the entire authority model
Introduce explicit state/diagnostics such as:
- `authority_mode`
- `authority_locked`
- `authority_ready`
- `rc_link_ready`
- `rc_takeover_ready`
- `autonomous_failsafe_ready`
- stable refusal reason

Names can differ, but semantics must be unambiguous.

## 3. RC_TAKEOVER policy
This mode means:
- Companion still owns the automatic Offboard sequence when authorized.
- A connected/calibrated RC link is a REQUIRED safety condition before Arm/non-zero thrust.
- RC loss before Arm => reject.
- An actual pilot takeover, PX4 mode departure from Offboard due to RC, Kill, or other explicit operator intervention must be treated as a one-way takeover event for that flight attempt: stop companion target/heartbeat as appropriate, latch `OPERATOR_TAKEOVER` (or equivalent), never automatically request Offboard again, and require a safe ground reset/new explicit preparation before companion control can resume.
- Do not implement code that fights the pilot by repeatedly re-requesting Offboard.
- Preserve PX4-native takeover rather than trying to synthesize manual stick control in the companion.

Because the receiver is NOT currently connected, RC signal mappings/kill behavior are not yet field-verified. Delivered config must therefore keep any RC-takeover confirmation gate false/unconfirmed by default.

## 4. AUTONOMOUS_NO_RC policy
This mode deliberately does NOT require RC link/manual signal as a precondition.

However, it MUST NOT be implemented by simply removing the old `manual_ready` check.

Instead require a separate explicit `autonomous_failsafe_ready`/confirmation contract at BOTH:
- state-machine takeoff/Arm admission
- final Arm publisher boundary
- final non-zero-thrust attitude publisher boundary

Default must be false/unconfirmed.

Do not automatically write or alter PX4 parameters. The companion may expose expected/confirmed configuration facts, but until a separately reviewed PX4 v1.16 failsafe contract is confirmed by the operator, autonomous Arm/non-zero thrust must remain fail-closed.

## 5. Preserve all existing 0.4.1 controls
Do not weaken:
- four flight gates at final Arm/non-zero publisher boundary
- stage-one exact-zero-thrust defense
- `flight_parameters_confirmed` requirement
- battery/preflight requirements
- attitude/Z/VZ freshness and validity
- PX4 Offboard confirmation
- publisher ownership/exclusive graph checks
- shutdown/session/operator lease checks
- envelope limits
- ACK correlation and command-400 transaction semantics
- stale socket/update hardening
- install-not-auto-switch-current behavior

Refactor `manual_ready` only as needed to introduce the new authority policy. Existing stage-one zero-thrust `attitude-ready` must remain usable with authority UNSET if that is safest, OR require authority to be selected before stage-one if you can justify it. In either design, Arm/non-zero thrust with authority UNSET is forbidden. Document the choice.

## 6. Takeover/recovery behavior is a first-class requirement
Add tests for:
- external nav_state leaves OFFBOARD while flight attempt is active
- RC takeover event
- RC loss in RC_TAKEOVER
- session disconnect
- heartbeat loss/fault
- Kill/termination-like safety indication if represented in available messages

The companion must never automatically re-enter Offboard after takeover/failsafe without a fresh explicit ground authorization.

## 7. No flight parameter invention
Keep all existing second-stage flight numerical sentinels/disabled gates unchanged unless they are already validated baseline values. Do NOT choose hover thrust, gains, max thrust, slew, height, speed, tilt or timeouts for real flight in this task.

## 8. Orange Pi compatibility
Keep the control core portable: C++17/rclcpp/POSIX, no GUI dependency, no x86-specific logic, no hard-coded `/home/jikl62`. Authority selection must work identically on PC and later Orange Pi. Do not install/configure Orange Pi in this task.
