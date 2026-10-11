# 0.4.2-authority-modes 验收矩阵

范围：实际新候选 Core/Runtime/最终 publication boundary、console 和 ROS adapter fixture。所有 Arm/nonzero 正向路径只到合成 sink；不构成真实飞行放行。命令：`bash scripts/test.sh`、`bash scripts/test_authority_harness.sh /tmp/authority-results.json`。

| # | 契约验收项 | 离线结果与证据 |
| --- | --- | --- |
| 1 | 默认 UNSET | PASS；authority_runtime、Runtime JSON parser |
| 2 | UNSET 上游 Arm admission 拒绝 | PASS；authority_runtime、harness unset_takeoff_denied |
| 3 | UNSET 最终 Arm 拒绝 | PASS；authority_boundary 的实际 Arm sink |
| 4 | UNSET 最终非零拒绝 | PASS；authority_boundary 的实际双 sink |
| 5 | 只在 verified safe IDLE/output-off 选择 | PASS；authority_runtime 的 stale/armed/airborne/rest/conflict/fault/output 等拒绝 |
| 6 | 开始准备后不能修改 | PASS；SELF_CHECK 就 lock；authority_runtime |
| 7 | 八个飞行阶段始终锁定 | PASS；authority_runtime 真实 REQUEST_ARM→REQUEST_DISARM 生命周期 |
| 8 | 地面完成、输出停、无 pending 后显式 reset | PASS；authority_runtime 完整完成及 pending/fault 测试；reset 回 UNSET/IDLE |
| 9 | 缺失/过期 RC 拒绝 Arm | PASS；authority_boundary、telemetry、review first；首帧/重复 raw 也拒绝 |
| 10 | RC confirmation false 拒绝 Arm/nonzero | PASS；authority_boundary + harness |
| 11 | 合成完整 RC 路线 policy allow | PASS；实际假 Arm/nonzero sink；harness + review first 的二次 raw 推进正向 |
| 12 | nav departure/明确接管锁存且不重入 | PASS；authority_runtime 覆盖 ACK 前 departure；harness；review counter 丢中间帧 |
| 13 | RC flight loss fail-closed/release | PASS；authority_runtime、harness takeover_rc_loss；恢复健康仍停止 |
| 14 | 自主不依赖 RC | PASS；manual/RC/switch readiness=false 的实际合成生命周期 |
| 15 | 自主未确认，最终 Arm 拒绝 | PASS；authority_boundary + harness |
| 16 | 自主未确认，最终非零拒绝 | PASS；authority_boundary 实际双 sink |
| 17 | 自主完整 policy 合成 allow | PASS；harness autonomous_without_rc_synthetic_arm_and_thrust |
| 18 | 自主不改 PX4 参数 | PASS；只读订阅/immutable confirmation；源码复核；部署/安装脚本原文一致 |
| 19 | stage-one exact zero 保持 | PASS；原 publication_boundary/attitude_runtime + 新 UNSET harness |
| 20 | stage-one 注入非零拒绝 | PASS；原完整回归、double denorm 与 float 编码防线保持 |
| 21 | 最终四门仍独立必需 | PASS；原 truth table + authority_boundary；交付 YAML 四门 false |
| 22 | ACK lifecycle / command400 顺序 | PASS；原 ack_lifecycle、完整 Arm→Disarm；ack/track 原文及 hash 一致 |
| 23 | socket/update hardening 保持 | PASS WITH CONDITIONS；受保护源码/文件权限一致；inode/flock/临时 symlink/service UNKNOWN fixture PASS；原生 AF_UNIX 受环境禁止，UNKNOWN |
| 24 | installer 不自动切 current | PASS；install 原文/hash 一致；临时 fake build/current fixture；未运行真实安装 |
| 25 | JSON 权限/锁/readiness/稳定原因 | PASS；真实 Runtime emitter→Python JSON parser；review diagnostic；即时接管/故障 readiness=false |
| 26 | 补充 G2 authority/takeover harness | PASS；直接包含候选真实 Core/Runtime/双 sink，24/24；重复 JSON 完全一致；原 G2 单独 39/39，仅旧基线 |

完整结果：12 个 C++ 程序退出成功（其中原生 socket 子检查 SKIP），adapter fixture syntax PASS；Python 18 run / 16 PASS / 2 native transport SKIP；四项审查回归 RED→GREEN；新 harness 24/24。原 frozen G2 单测 5/5 和 39/39 只属于 0.4.1。

非离线证据保持 UNKNOWN：0.4.2 真正 Ubuntu22.04/Humble build/link、精确 pin 的新消息集成、真实 DDS graph/timing、RC 校准/Channel5 Kill/Channel6 Arm/physical takeover/native override、autonomous failsafe 实际动作、Orange Pi/aarch64、飞行调参/动力学。0.4.1 已有现场 build 和 disarmed stage-one READY 的事实按 handoff 保留，但不转移为新版本 PASS。

**真实 Arm、Takeoff、非零推力 NOT AUTHORIZED。** 未连接 FC01、未启动真实 ROS/Agent 控制链、未写 PX4 参数、未切真实 current。
