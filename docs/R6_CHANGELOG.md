> 历史 R6 基线记录，保留原结论与来源。本文件不是 R6.1 的当前结论；请先读根目录 `README_FIRST.md` 和 `docs/R6_1_SAFETY_EVIDENCE.md`。

# FC01 M1 智能体 B：R6 变更记录

版本：`0.4.4-offboard-flight-dev-r6-control-recovery-dev`。开发基线：交接包内完整 R5。
状态：**DEV / BLOCKED；尚未关闭全部飞行 P0，不是带动力自主飞行版本。**

## 已实施

1. 原 `DESCENT_TIMEOUT → SAFETY → 空输出` 改为 `RECOVERY_DESCENT`。
   复用原垂直控制器的限幅、积分约束和物理 slew 状态，目标只能继续向地面下降。
   第一次超时建立一个绝对恢复期限；重复 LAND、掉线、shutdown、反弹均不能续期。
   默认 15 秒是待审查的监督策略上限，并非真机动力或安全降落时间的实测结论。
2. 新增可选 `PX4_LAND_PENDING`。必须同时有显式外部 Land 后备验证记录、当前有效本地
   XY/VXY/Z/VZ、可选 Land 模式及原有健康/Authority/门控，才计划一次 Command 21。
   ROS 最终发布 guard 再次检查这些条件；模式切换前持续双 sink；21 不重复提交。
   21 ACK 不能证明切换模式，更新后的 AUTO_LAND 也仅证明观察到模式，不能证明任务可运行。
3. 恢复期限耗尽或估计/姿态/RC/Kill/发布/调度失败后，明确记录
   `PX4_FAILSAFE_PENDING_UNKNOWN_AIRBORNE` 等不确定责任。停流后不保留
   `CONTROLLED_DESCENT` 或“400 guarded recovery”的误导标签。
   Safety 下只读观察独立的新 status/land/Z/VZ；连续新 land 样本驻留后才可记录已接地解除解锁。
4. 节点在 target、heartbeat 每个实际本地 publish 返回后分别计数，记录间隔及部分失败。
   target 成功后再次采样最终 guard，heartbeat 失败不允许继续提交 176/400/21。
   每 tick JSONL 包含 monotonic、前后 phase、PX4 状态/新鲜度、Z/VZ、计划及实际提交推力、
   双 sink 计数、deadline、21 与 400 事务诊断。日志明确不证明 PX4 接收。
5. 新增无桨阶段**本地文件检查**脚本，不连接 ROS/串口，不发送任何命令。
   正式默认配置及两个 R6 独立配置四重门控全部 false，RC/自主后备确认 false；
   hover/min/max/slew/gain 为 -1 哨兵，防止把 S10 的仿真推力值作为真机校准值。

## 保留及透明调整

- R4 的 ACK 归属/跨事务 Command400 历史、单 publisher、RC/Kill、Authority、DDS 与原最终
  发布 guard 均保留。Command400 逻辑没有放宽，不用重启消除 ACK 歧义。
- S10/R5 的 1Hz 联合离地、新时间戳、两帧、反弹、最大采样间隔和 ASCENT/LIFTOFF 判据保留。
- 所有原始 R4/C 反例、C-S10 原文件按字节保留。原 B 的 S10 测试中只有预期旧断流行为的
  timeout 段随新有限恢复更新；原文件完整保存在 `tests/historical_r5/`。
- 增量补丁以 R5 原目录为基准，新增文件也在补丁中。`workspace/src/px4_msgs` 不修改。

## 独立开发审查

一个独立只读开发审查发现两项重要问题：停止控制后分类仍称受控；本地日志审核漏掉双 sink
静默尾段/异常。新增失败回归后修复。计划/已提交推力在部分失败后的日志丢失也补充验证。
另一个 Core API 假设保留：纯 fixture 如果让同一 status stamp 改写 nav，可触发通用退出描述；
实际 Node 使用严格 `Stamp::accept` 拒绝这种缓存改写。此项交 C 复核，不能视为实机证据。

## 尚未完成

延长 C 的健康 Armed+Offboard 检查超过整个恢复预算，约 38.05 秒仍为 RED：
恢复耗尽且没有实际 PX4 后备证明。无 XY 场景默认不请求 Land21。
**这不是完整 P0 修复；立即超时断流缺陷得到修复，有限恢复后的物理控制权保证仍 BLOCKED。**
真实 Humble 编译/链接、DDS、PX4 飞行任务执行、FC01 硬件与带动力飞行均未执行。
不重复 X500 调参，不采用其动力结果给真机填默认值。
