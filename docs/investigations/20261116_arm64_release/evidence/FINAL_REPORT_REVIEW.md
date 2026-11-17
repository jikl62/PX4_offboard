# Final delivery documentation/evidence read-only review

2026-11-16. Reviewed FINAL_REPORT_zh.md, DEPLOY_zh.md and evidence/vm/RESULT_zh.md against actual dist/arm64/DELIVERY.json and raw VM records. No production/report edits, service operation or test rerun by reviewer.

Result: no blocking factual mismatch found. Two small documentation polish items below; package/control identity, preserved config, actual VM vs physical hardware boundaries and previous-version automatic rollback limitation are accurately stated.

## Checked facts

- Actual archive6497136bytes, SHA2563c876f6f98d45df994658ad3f68691c12dee0cf14befa18f2824a52f58d2a123; installer7b35de4b7e498d46c56ac7b9abbf032c2ba64d1a76ab11177f9391bc4fc4e956; source56424d7f16af2b6c2a93eb121e0b2bb00a6fb34a; controlcandidateadc55d0b7c67e80bf9d77b1082208c898f95b756. All agree with DELIVERY and both report identities. Actual SHA/syntax/source pin previously independently checked.
- Final actual-install7 succeeds, repeat uses samebackup and PID4623, rollback restores oldPID5339. Each captured controller_count1. Independent comparison of complete captured runtime: install/repeat change ONLY fc01_entry_version and fc01_config_source_path; manualrollback equals complete original dict. Original/final runtime bytes shareSHA929eb1cb029d3a6af38197e0d8e396ca9145a0d5b83a8465dd049f715cefe13c. Target1.23m/RCslot1 retained.
- Auto recovery was previousb4392e37 package with30sec deadline, not deliberate finalpackage failure. Main FINAL_REPORT explicitly states this; RESULT table says old30s. Nativecode0/failed_restored journal evidence present. Final180s candidate installed/repeated/rolled back separately. No claim finalpackage synthetic rollback was physically repeated.
- Hardware caveats explicit: QEMU-system ARM64 Ubuntu22.04/systemd249/Humble; generic5.15 kernel, not actualZero3W; directDDS synthetic disarmed feedback; PTY dummyAgent notXRCE. Old/new use same compiled candidateELF in different trees, so transactiontest not historical-flightbehaviorcomparison. User still performs actual hardware installation.
- Run logs support69 host relatedunit tests and43 actualguest unit tests; these backend-injected tests are distinguished from realPID1/systemctl integration. Wire_summary42083setpoints/heartbeat,0nonzerothrust,0commands accurately reflected;78.71s intentional shutdown gap explicitly not flightheartbeat bound.
- Actual userstatus commandlog now complete: installer--status exit0; familiar navigation.py status exit0 displaying READY_WAIT_ARM/1.23m; repeated rollback after alreadyrolledback refuses exit1 with currenttransactionmismatch. This is expected idempotent safety refusal, not another failed install. No real board operation claimed.
- DEPLOY commands target delivered currentversion filenames, ordinaryuser, desktop tarargument, statecheck and backupreadfromINSTALL_STATE then rollback. No oldSDK/controller path used. Required active oldservice/liveground/RC prereqs are stated. Explicit no wholeinstaller sudo; supervised sudo only.
- All relative Markdown targets in the three reviewed docs resolve. VM SHA256SUMS94entries all match, no missing/extra files at reviewtime.
- `git diff adc55d0 -- native/src native/config native/tools launcher.py standby.py navigation_service.py` empty at reviewtime, consistent no productioncontrolchanges claim.

## Nonblocking documentation polish

1. evidence/vm/COMMANDS_zh.md still contains stale parenthetical placeholders '下文待填' and '成功后补结果', despite actualbackup/success now present. Remove those phrases to avoid suggesting incomplete evidence.
2. FINAL_REPORT/RESULT could link the newly completed evidence/vm/user-status-and-repeat-rollback.log and spellout statuscommands exit0, secondrollback expectedrefusal exit1. Existing reported installationfacts remain valid. `navigation.py monitor` is an inherited readonly command; separate actual monitorinvocation is not in this newlog, so avoid claiming all listingcommands independently reexecuted if notdone.

## Sensitive-data scan

Read all report/evidence regularfiles and contents of allthree tar.gz archives and gzip logs,164plain/archive memberfiles total,54archive members. No privatekey PEM/OpenSSH headers, GitHub/OpenAI/AWS tokenpatterns or credential-bearing URL patterns found. No id_rsa/id_ed25519/vm_id/authorized_keys/*.pem archive names found. This is scoped patterninspection of these actual deliveryrecords, not a claim to mathematically prove absence of every possible secretencoding. VM SSH privatekey remains outside supplied evidence.
