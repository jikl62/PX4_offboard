> 历史 R6 基线记录，保留原结论与来源。本文件不是 R6.1 的当前结论；请先读根目录 `README_FIRST.md` 和 `docs/R6_1_SAFETY_EVIDENCE.md`。

# 智能体 C：R6 独立复审任务书

对象：`0.4.4-offboard-flight-dev-r6-control-recovery-dev` 完整源码及外部 SHA256 清单。
唯一基线：交接包内 R5 完整源码。**本候选目前 DEV / BLOCKED，无飞行放行申请。**

请独立重算哈希、应用 R5_TO_R6.patch 并逐文件比对，不改 B 源码、原始 C 反例或期望。
必须区分离线审查、真机无桨命令、动力台架及自主飞行，不用一个 PASS 覆盖这些阶段。

## 立即检查

1. 阅读 `R6_CHANGELOG.md` / `R6_SAFETY_CLAIMS_AND_EVIDENCE.md`，独立运行 `scripts/test.sh`。
   两个 AF_UNIX 跳过属于 UNKNOWN。B 自测和开发审查不等于 C 结论。
2. 保持原 C-S10 `C_S10_POSITIVE_GATE.cpp` 完整字节，分别编译 R5/R6，运行 timeout/shutdown。
   R5 RED，R6 原观察窗口 GREEN。继续运行新 `C_R6_EXTENDED_HEALTHY_GATE.cpp`：
   **当前仍 RED**，约 38.05 秒恢复预算耗尽。不得将其忽略、改反馈或当作批准真机的 PASS。
3. 独立检查 RecoveryDescent/Px4LandPending：入相、同周期输出、下降目标、原 slew/限幅，
   LAND/掉线/shutdown/反弹不续绝对 deadline；不能因为关机或 ACK 超时停掉健康输出。
4. Land21 必须经过有外部记录的合同及当前 local position/velocity、available mask、Authority、
   四门控和最终 guard。提交只能一次，ACK不能确认模式，新模式也不能确认 flight task。
   无 XY/失估计时拒绝把 Position/Land 当可靠独立后备，不改真机参数。
5. 原 R4 400 ACK 隔离/C11-C12、未提交/已提交 Disarm 反弹、迟到 ACCEPTED/FAILED/UNKNOWN、
   state-only ground terminal、新 status/land 证据全部复验。故障后无重复 routine Disarm 或新 Arm。
6. Node 的 target 与 heartbeat 每个 sink 前后独立检查。target 成功/heartbeat 异常、target 异常、
   控制 stall 后不得提交 176/400/21；日志保存计划与已提交推力、两个 commit 和间隔，不能称原子提交。
7. 覆盖 Z/VZ stale/reset、姿态 invalid、battery/failsafe、RC/Kill、Publisher/Agent 断开、console失联、
   SIGINT/TERM/KILL/崩溃及 PX4 主动改模式。明确输出能力、责任和外部证据，不能残留受控标签。
8. 正式 default config/R6 两套独立 config 全部默认 gates=false，动力参数 unset；
   历史 S10 仅模拟配置不得作为部署配置。无桨脚本只读本地文件、不发命令。

## 必须补的外部证据

- Ubuntu22.04 / ROS2 Humble 原生编译、链接、厂商 pinned DDS 接口检查。
- 实际 FC01 固件/传感器/参数指纹一致的独立后备验证：停止心跳后 PX4 实际模式、
  task 可运行性、估计前提、操纵链和动力状态。不能仅看 can_set mask、ACK、nav_state。
- target/heartbeat/command 的真实本地日志与 PX4 ULog 对齐。本包仅离线数据，不声称有这些证据。
- A 的真实硬件适配要求、实测动力/电池、RC/Kill 与地面证据。
- 同 stamp 人工改写 mode 的 Core API 攻击：Node Stamp.accept 应拒绝该输入；请核对所有实际入口。

## 建议交付

`C_R6_INDEPENDENT_REVIEW.md`、`C_R6_TEST_MATRIX.csv`、新肯定式独立反例、
`C_R6_RELEASE_VERDICT.md`，分别标离线 GO/NO-GO、无桨阶段准入及 powered flight NO-GO/GO。
无法证明恢复耗尽/硬终止的外部控制权时，保持 **NO-GO for powered autonomous flight**。
