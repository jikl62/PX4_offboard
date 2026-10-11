# R6 control recovery design

Binding brief: uploaded `02_AGENT_B_R6_FIX_PROMPT.txt`, SHA256
`2dcfef14af46b24ebe53725362fd9fdb42f0358bb2dbeacc7b780083e9d4334b`.
User explicitly authorized immediate offline implementation in an independent R6 copy.

R5 Descent timeout calls fault(), returns an empty Output, and stops both sinks
with healthy armed Offboard telemetry. Replace this path with RecoveryDescent.
Keep the same vertical controller slew state, downward target, limits, detector
dwell, ACK tracker, authority lock and final publication guard. Recovery has one
absolute deadline that LAND/disconnect/shutdown/bounce cannot renew.

Optional Px4LandPending may request a single NAV_LAND (21) only after an explicit
previously validated fallback contract and current fresh local position/velocity
and selectable Land mode. Keep both sinks while PX4 remains Offboard and healthy,
within the original recovery deadline. Do not treat ACK as a mode transition;
new post-commit AUTO_LAND status is observed ownership, not proven task execution.
No routine Disarm until original ground proof. Do not repeat command 21.

Failure to control (telemetry, envelope, stall, sink failure), or expiry without
observed handoff, is disclosed as PX4_FAILSAFE_PENDING / UNKNOWN_AIRBORNE.
There is no universal safe output if both bounded recovery and independent
fallback fail. Real FC01 fallback and powered autonomous flight remain BLOCKED
until external evidence and C review close that gap.

Add per-tick monotonic phase events, separate actual local target/mode commit
counts/gaps and partial exceptions. Local publish success is not PX4 receipt.
Default flight gates, fallback confirmation and real gains remain disabled/unset.
Deliver offline checks only; no hardware writes or powered launch script.
