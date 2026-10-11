# R6 → R6.1 变更说明

日期：2026-10-10。按用户授权，仅修复恢复预算耗尽主动停流，不重做已验证的 DDS/Command176/基础 ACK/Offboard 准备/地面释放。

## 生产修改

1. `attitude_altitude_core.hpp`：保持原 `aa_recovery_seconds=15` 和一次绝对截止时刻，将截止耗尽从 `fault(); return {}` 改为锁存告警。每周期仍经过原健康、Authority、gates、参考/reset、Z/VZ jump、速度/高度、tilt 检查，并沿用原反馈下降与饱和检查和最终发布策略。未新增备用推力数值、自动 Arm/Disarm/21/176，也未改任何控制器增益。
2. 同文件增加只读 `recovery_budget_exhausted()` 和明确的逾期责任标签：`COMPANION_GUARDED_DESCENT_ACTIVE_RECOVERY_OVERDUE_HANDOFF_UNVERIFIED`。日志理由为 `RECOVERY_BUDGET_EXHAUSTED_GUARDED_CUSTODY_ACTIVE_HANDOFF_UNVERIFIED`。截止时刻、进展、21 提交/模式、400 ACK 继续分别记录。
3. `fc01_system_node.cpp`：仅在现有 `phase_event` 和状态 JSON 增加逾期 boolean；不改 publisher、DDS topics、消息 schema、Command176、ACK、signal handler 或退出控制流。原 Node 关闭 60s 后健康 Armed+Offboard 继续输出的规则沿用。
4. `VERSION` 和当前文档更新。三份真实/离线 YAML 仅调整截止参数注释，**解析后的全部参数值与 R6 一致**。历史 R6 报告显式标注历史；历史源码校验表存档。

## 测试与可追溯性

新增 `tests/test_r6_1_recovery_custody.cpp`：正常、正常关闭、console 断连三种情况下各观察 600 秒虚拟时间；每周期实际调用 Runtime 与 publication boundary，肯定式检查两条 sink、限幅、slew、截止不能续期、不能转回上升、无盲目命令。新增 13 类逾期后失效、最终 guard、模式退出和晚到 1Hz touchdown→state-only Disarm→原 ground release。

原 C-S10、C 扩展和 R4/C11/C12 源码均不修改。原 `C_R6_EXTENDED_HEALTHY_GATE.cpp` 的注释保留原历史背景；其实际执行结论以当前日志为准。

四份旧 B 测试原始 R6 字节保存在 `tests/historical_r6/*.original`：

- `test_m1_r6_recovery.cpp`：替换过时的“预算到期必须 Safety”期望；External Ground 观察改从真实 invalid-Z 故障进入 Safety。扩展 21 ACK 接受/拒绝/缺失观察到整个预算之后，保留 no-retry、真正 final commit 和 sensor 前提。
- `test_m1_s10_1hz_recovery.cpp`：仅修改超时终态的旧期望，原 1Hz/freeze/bounce/高度+速度/stamp 判据不变。首轮完整回归因此得到 exit134，保留日志而不隐去。
- `r6_fault_child.cpp` 与 `test_r6_process_failures.py`：正常 OS 信号后要求预算过期仍保持健康输出，替换旧 Safety 期望；SIGKILL/crash 仍不宣称可恢复。40 tick 是 OFFLINE fixture 观察窗口，退出不是生产 Node 的关闭成功证据。

R6.1 不修改 Runtime/Command400 ACK、Authority、vertical_controller、publication_guard、publication_boundary，以及 235 个 px4_msgs 文件。源码交付保留完整 R6 工程，其余文件沿用。

## 取舍与范围

完成期限和是否仍持有控制权是不同的条件。在当前无 XY 且未证实 PX4 备用能力时，无法同时从代码证明“到固定时刻必须停流”和“仍 Armed+Offboard 时已有可靠接管”。本版取消前者作为单独停流理由，保留并锁存逾期诊断。输出幅值、slew、速度、估计及 authority 继续受限；**总恢复时间不再由该计时器限定**。不把这一取舍称为物理安全验收。

悬停精度、水平保持、动力标定、估计器优化、自动失流后备和真实飞行验收均没有新增实现。
