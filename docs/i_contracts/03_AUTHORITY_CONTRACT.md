# Authority-mode safety contract

## State model
Suggested conceptual state:

`UNSET -> selected(RC_TAKEOVER | AUTONOMOUS_NO_RC) -> LOCKED -> flight attempt -> SAFE_GROUND -> selectable`

Authority selection is orthogonal to PX4 nav_state. It is an application safety contract.

## Required invariants
1. `UNSET` can never reach Arm or non-zero thrust.
2. Authority mode is immutable during an active preparation/flight attempt.
3. Final publisher boundaries re-evaluate authority eligibility; upstream checks alone are insufficient.
4. RC_TAKEOVER and AUTONOMOUS_NO_RC have different predicates. Do not encode them as `manual_ready=true/false` shortcuts.
5. A pilot/failsafe takeover is one-way for the attempt; companion must not automatically reacquire Offboard.
6. Selecting AUTONOMOUS_NO_RC never writes PX4 params and never implies that its failsafe contract has been verified.
7. All new confirmation flags default false except neutral observability fields.
8. Existing four flight gates remain false in delivered configuration.

## Suggested refusal reasons
Stable reason enums/strings should distinguish at least:
- AUTHORITY_MODE_UNSET
- AUTHORITY_MODE_LOCKED
- RC_LINK_REQUIRED
- RC_TAKEOVER_CONTRACT_UNCONFIRMED
- AUTONOMOUS_FAILSAFE_UNCONFIRMED
- OPERATOR_TAKEOVER_LATCHED
- AUTHORITY_CHANGED_DURING_ATTEMPT

Exact names are implementation-defined.
