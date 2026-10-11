# R6.1 独立开发代码复审记录

本次独立审查对象：完整 R6 与 R6.1 候选目录的生产 diff、原 C 断言、历史 B 测试存档、新回归及当前四份说明。审查者只读，没有操作 FC01 或修改源码；此记录不是正式智能体 C 的物理飞行签署。

独立审查结论：未发现 Critical 或 Important 实现缺陷。预算逻辑之外的健康、Authority、reset/jump/tilt/高度速度包络、饱和与 final guard 没有绕过；原 Runtime、ACK 和地面释放路径沿用。原 C-S10/扩展源、四份历史 R6 B 存档独立字节核对一致。

独立验证：严格 C++17 编译；原扩展门禁基线在 38.05s RED、候选 GREEN；新三组 600s/13 fatal/final guard/mode exit/晚到落地测试候选 PASS，同新测试在 R6 上按预算耗尽原因失败。额外逾期后 saturation 和 excess tilt 注入均按原规则 Safety、无命令、无 sink 提交。

发现并已修复的一项 Minor：逾期后 `Landed` 等待 `at_rest` 的 ramp-to-zero 阶段被泛化理由写成“下降中”。新回归先复现 RED，再将通用理由改为 `RECOVERY_BUDGET_EXHAUSTED_GUARDED_CUSTODY_ACTIVE_HANDOFF_UNVERIFIED` 并获得 GREEN。独立后续审查重新编译和执行新回归，包括 zero-thrust、at_rest 未确认、禁止 Disarm 的诊断用例，确认问题关闭。

对当前 `README_FIRST.md`、`R6_1_CHANGELOG.md`、`R6_1_SAFETY_EVIDENCE.md`、`C_R6_1_REVIEW_TASK.md` 的后续阅读没有发现物理发布过度主张；总保管时间不受截止计时器限定、无效下降/独立后备未证明和飞行 NO-GO 均写明。

当前评估只支持 timer-caused voluntary custody loss 的离线代码修复。打包与原生/实机结论分别按证据报告，不以审查文字代替实际验证。
