# ARM64 release builder independent review

2026-11-16. Read-only review of build_arm64_bundle.py and its actual source closure/build-evidence plan. No production/source edits, commits, service/hardware operations.

Current result: no unresolved blocking defect found after parent fixes below. Evidence coverage, directory-link rejection and SHA pin assertion are implemented and reviewed. Latest builder+installer+service suite38 tests PASS, py_compile and git diff --check PASS. No claim of completed production package readback or ARM64 runtime validation by reviewer.

## Confirmed properties

- Source comes from git archive HEAD, not arbitrary worktree/untracked bytes; selected native source/tool bytes are compared with control_candidate commit. New installer/service scripts must be committed before build or missing committed input refuses.
- Included runtime closure uses existing names(source) with three explicit new prebuilt files. Selected payload contains normal files; tar writer creates only explicit regular-file members under validated FC01_OrangePi_<version>_arm64 top directory.
- ARM64 ELF architecture is verified while copying installed artifacts. File symlinks resolve only inside build workspace source.parent and become ordinary copied files.
- Manifest binds every emitted immutable file to SHA256, architecture/Ubuntu/ROS/version/controller path. Final archive metadata is deterministic and immutable manifest is read back through production bounded extractor with full file set/hash/ELF checks.
- install_fc01.sh embeds committed installer source, replaces exact expected-sha argument with fixed default+choices for this archive, preventing CLI alternate hash. Actual current replacement target exists exactly once. Delivery records archive/installer SHA and source/control commits.

## Findings resolved

1. Initial build-evidence plan hashed all18 native/tools whereas runtime closure contains6; builder would fail sha(entry/missingtool) after actual build. Parent has already changed /tmp/fc01-arm64-build-evidence-plan.py to actual CMake inputs: native package complete tree plus4 installed tools. This resolves plan/closure mismatch; no source algorithm changes needed.

2. Original one-way evidence validation accepted partial maps. New validate_evidence requires exact package-source plus four installed-tool inputs, exact nonempty copied install file set, controller/checker entries, safe relative paths/SHA256 and ARM64/Ubuntu22.04/Humble identity. Bytes are then checked. Added test validates complete fixture and rejects omissions and changed output bytes.

3. Original copy_install silently skipped directory symlinks. It now explicitly rejects them before is_dir continue; file symlinks remain bounded and copied as ordinary files. New unit test and reviewer temporary-tree reproduction confirm directory symlink rejection.

4. Fixed SHA injection now asserts exact source replacement target appears once, then injects fixed default+choices. Current real installer target verified. Refactor mismatch refuses packaging.

Additional note: source archive extractall operates on local committed Git tree, not user release tar; production install extraction is separately bounded. Never repurpose this trusted-source extraction for untrusted packages.

## Verification/evidence

- Derived curated names(root) native/tools set:6 versus initial full plan18; listed12 missing tools in findings file. Revised plan4 installed tools subset now present.
- Source inspection of generator pin target/current installer argument exact match.
- Added tests/test_arm64_bundle.py has2 tests for complete/partial/changed evidence and directory symlinks. Reviewer ran these plus installer30+service6:38 PASS. Final production builder path still performs bounded extractor readback; parent must report actual artifact results separately.

Final reviewed parent checks landed at21:18. Real ARM64 install/ldd/typesupport and real-systemd VM validation are separate parent tasks.
