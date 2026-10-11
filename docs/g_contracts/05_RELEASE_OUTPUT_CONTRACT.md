# Release output contract

Deliver exactly these primary artifacts:

1. `fc01_system_v0.4.1-safety-hardening_source.tar.gz`
2. `fc01_system_v0.4.1-safety-hardening_source.zip`
3. `fc01_system_v0.4.1-safety-hardening_vs_0.4.0.patch`
4. `G_DELIVERY_REPORT_0.4.1-safety-hardening.md`
5. `SHA256SUMS.txt`
6. `PACKAGE_VERIFICATION.txt`

Requirements:
- `VERSION=0.4.1-safety-hardening`
- `package.xml` version `0.4.1`
- immutable new release semantics
- install builds candidate but does not switch real/current
- no compiled binary/build/install/log/cache in source archives

## G report structure
1. Files actually read
2. D I-01 root cause / boundary analysis
3. Exact hardening design
4. Dangerous publisher boundary table
5. Stale-socket design and race analysis
6. File-by-file diff summary
7. New/old tests and raw results
8. Preserved baseline invariants
9. Patch replay/archive verification
10. UNKNOWNs and unverified hardware behavior
11. Explicit statement: this release does NOT authorize real Arm/Takeoff
