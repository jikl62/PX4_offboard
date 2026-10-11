# G1 whole-change review

Author review completed against frozen 0.4.0 and the formal G contract. This is not an independent review. The requested fresh review agent did not run: it returned a usage/quota limit before producing any review findings. Independent review status is UNKNOWN.

Reviewed final Arm sink, paired attitude/mode sink, Runtime context construction, phase transitions, lease/health checks, four-gate status, UnixServer constructor/destructor, shared flock, updater/service checks, install invariants, and tests.

- Every Arm 400/param1=1 call reaches the unconditional guard in publish_command(), including flight/sim callers. Non-Arm command fields remain unchanged.
- Every attitude target and its heartbeat reaches publish_attitude_at_boundary(); stage-one nonzero is tested before conversion to float. Policy/encoding denial calls neither sink. Node latches the diagnostic, clears output and faults the attitude core; a second request cannot clear the latch.
- IDLE/SELF_CHECK have no normal publication. PRESTREAM/REQUEST_OFFBOARD/OFFBOARD_READY permit exact zero with gates false. A synthetic unknown phase originally exposed a permissive zero-output case; its RED assertion was observed and the exhaustive phase allowlist fixed it.
- RequestArm zero output can remain armed while waiting for ACK; an Arm send itself still requires disarmed. DESCENT/LANDED may continue after lease loss under the original healthy emergency descent semantics. Shutdown denies all attitude output, deliberately releasing control rather than maintaining a nonzero shutdown descent.
- A target/mode pair is not a DDS atomic transaction. Sink exceptions propagate out of the node; target-first ordering prevents a failed target call from issuing its heartbeat, but in-flight/previous DDS samples and asynchronous signals remain unverified.
- UnixServer never removes an existing socket on startup. Normal cleanup is limited to its own newly bound owned socket dev/inode with the same lock identity. It never unlinks the lock pathname.
- Update never connects to a controller, deletes a socket, kills a process, stops/starts a service or launches a task. It switches only under the same advisory lock, with repeated service/path/lock checks. The lock coordinates 0.4.1 participants; legacy 0.4.0/manual noncooperating actors require explicit human quiescence. No proof is claimed against malicious same-UID filesystem replacement or arbitrary service restart.
- All 26 protected candidate files and the original ack()/track() function bytes match; test/install, source-archive and replay evidence is recorded separately.

No additional demonstrated implementation defect remained in this author review. Real Humble build/link, native AF_UNIX transport, DDS and flight behavior remain UNKNOWN; review approval for hardware use has not been given.
