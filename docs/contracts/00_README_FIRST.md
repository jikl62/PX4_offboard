# FC01 0.4.0 Attitude-Altitude — F 智能体交接包

## 你现在要做什么

这是给“智能体 F：0.4 实现工程师”的完整输入包。请先完整阅读：

1. `01_F_TASK_SPEC.md` —— **最高优先级任务书**；
2. `02_VERIFIED_BASELINE.md` —— 已由真机验证的事实，禁止重新猜测；
3. `03_IMPLEMENTATION_CONTRACT.md` —— 0.4 必须满足的软件合同；
4. `04_ACCEPTANCE_TESTS.md` —— 交付前必须通过的测试；
5. `05_PORTABILITY_CONTRACT.md` —— 为 Orange Pi 迁移预留的硬约束；
6. `reports/` —— A/B/C 三个独立智能体的完整原始报告；
7. `baseline_source/` —— 已在用户 Ubuntu + ROS2 Humble 上构建并真机验证的 0.3.1 基线源码；
8. `manuals/` —— DM-FC01 与 Orange Pi Zero 3W A733 手册。

## 权威性排序

发生冲突时按以下顺序处理：

1. 真机已验证事实（`02_VERIFIED_BASELINE.md`）；
2. 用户提供的实际 0.3.1 基线源码；
3. PX4/px4_msgs **v1.16** 的源码与接口；
4. DM-FC01 手册；
5. C 架构报告；
6. 其他推论。

不要拿 PX4 `main` 的消息结构覆盖 v1.16；不要为了“看起来更现代”重写已经通过真机验证的 ACK transaction。

## 交付原则

目标不是写 demo 片段，而是产出一个新的、可审查、可回滚的 `0.4.0-attitude-altitude` source release。第一阶段必须绝对禁止自动 Arm/Takeoff；第二阶段代码可以实现，但默认关闭，等待主控审查后才启用。
