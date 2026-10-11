# Release output contract — Agent I

Deliver a NEW candidate, never modify 0.4.1 in place:

- `fc01_system_v0.4.2-authority-modes_source.zip`
- `fc01_system_v0.4.2-authority-modes_source.tar.gz`
- `fc01_system_v0.4.2-authority-modes_vs_0.4.1.patch`
- `I_DELIVERY_REPORT_0.4.2-authority-modes.md`
- `SHA256SUMS.txt`
- `PACKAGE_VERIFICATION.txt`

Requirements:
- VERSION=`0.4.2-authority-modes`
- package.xml version=`0.4.2`
- delivered YAML keeps all real flight gates false and unresolved flight numerical parameters at sentinel/unconfirmed values
- source archive contains no build/install/log/cache/ELF
- patch cleanly replays on pristine supplied 0.4.1 and replayed tree byte-matches delivery
- installer may build an immutable release but must never switch real `current`

Report must explicitly state UNKNOWN items and that real Arm/Takeoff/non-zero thrust is NOT AUTHORIZED.
