# 开发 RED / GREEN 证据索引

以下日志记录真实缺陷或缺失功能的可执行 RED，不把编译 fixture 的拼写错误当作安全缺陷证明。最终结果由 `I_TEST_OUTPUT.txt` 和当前 tests 源码给出。

| RED 文件（docs/i_red_green/） | 所隔离的行为 | 最终 GREEN |
| --- | --- | --- |
| AUTHORITY_BOUNDARY_RED.txt | 基线完整 Arm context 无 authority 可通过假 sink | test_authority_boundary |
| AUTHORITY_RUNTIME_RED.txt | 旧 manual 条件满足时 UNSET 上游 takeoff 被接受 | test_authority_runtime |
| AUTHORITY_RECOVERY_RED.txt | 释放旧准备后未经显式 reset 即可再次 prepare | test_authority_runtime |
| AUTHORITY_CONSOLE_RED.txt | 缺少 authority parsing/one-shot guard | test_authority_tools |
| AUTHORITY_TELEMETRY_RED.txt | 缺少单独的 RC/switch observation helper | test_authority_telemetry |
| AUTHORITY_DIAGNOSTIC_RED.txt | battery/preflight 原因仍错误含 RC/manual 语义 | BATTERY_PREFLIGHT_REQUIRED regression |
| KILL_RETENTION_RED.txt | unmapped switch 可以清除已知 Kill | test_authority_telemetry |

审查的四项 RED 在 `I_REVIEW_REGRESSIONS_RED.txt`，对应 current `test_authority_review.cpp`。改动完成后四项 targeted GREEN 和完整套件都运行成功。初始审查输出在 `I_REVIEW_ORIGINAL_REPRO_OUTPUT.txt`；它保留修正前的失败事实。
