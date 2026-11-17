# 阶段 2 独立审查：姿态重置与心跳发布

审查日期：2026-11-16。审查基线：`3677b9dd1f35e092e43501d26164c6263545c947`。本子任务未修改生产源码、运行配置、既有报告或用户未跟踪文件。

## 结论

没有发现足够证据支持在本轮改变活动期姿态重置或部分发布拒绝规则。阶段 1 的 `reset_counter+1`、相同四元数会触发停流，是现有明确保护合同的可复现结果，不能直接升级成已确认误报。没有当前同次异常试飞的完整 ULog、companion 日志和可靠时间对应点，不能断言现场 `No offboard signal` 的先因。

### 活动期姿态重置

- `attitude_reference.hpp:37-52`：先检查四元数；已有参考且活动期 reset 变化时返回 `ActiveReset`，不偷偷更新 yaw/reset。普通 `q/-q` 数值符号变化但计数不变不会触发此门。
- `attitude_altitude_core.hpp:1068`（行号可能随本轮改动移动）：排队 reset/无效四元数先交安全扫描；`Core::step` 中重复检查当前参考。后续健康尾帧不能清锁存故障。
- `fc01_system_node.cpp:954`：每类排队事件以本轮基础快照覆盖本来源事实，姿态事件携带 q/reset，但没有独立保存其采样时刻、delta_q_reset。这个信息缺口限制事后因果判断；不是足以删除 reset 检查的证据。
- RC 准入路径从 `RcNavigationStart` 的已证明地面锚点继承 q0/reset；活动时不能用当前姿态重新建立参考绕过异常。
- 匹配 PX4 v1.16 源码 `src/modules/mc_att_control/mc_att_control_main.cpp:324-340` 显式消费 `delta_q_reset`，且在姿态估计 timestamp 晚于已有目标 timestamp 时调整已有目标。FC01 当前没有消费这个增量，因此只看最新 q 数值相同不能证明增量无害、参考连续或中间无重置。不能据此实现静默续发旧参考。

### 心跳与部分发布

- `publication_boundary.hpp:111-136`：目标前重新取上下文；目标 sink 正常返回后才记 target_committed；心跳前再取上下文。第三次检查拒绝产生 `HEARTBEAT_FINAL_CONTEXT_CHANGED_<reason>`，它表示目标已本地提交而心跳尚未提交的真实部分发布，不能当作正常完整未发送。
- `SystemNode::tick:398`：每轮清空 last_attitude_output，仅双 sink 成功才回记输出和 RC 准备流证据；异常锁存边界故障，禁止继续关联动作。
- `fc01_system_node.cpp:2267-2284` 使用单线程执行器，回调接收后才 tick；同一进程的订阅回调不会插到两个 sink 之间。但 TTL、周期预算、源年龄、图发现等仍可在三次取样间改变，不能据单线程取消最终检查。
- 匹配 PX4 `offboardCheck.cpp:46-51` 直接按 COM_OF_LOSS_T 和消息 timestamp 判定心跳新鲜度；`modeCheck.cpp:133-140` 才报告 No offboard signal。本地 publish 返回不证明 PX4 收到或采用。停流可形成失联，但没有同次日志不能断定现场先后。

## 实际回归

命令在项目根目录执行，除 Python unittest 进入 native/tests。所有测试均为开发机离线夹具，无 ROS 通信、串口或动力操作。

```bash
g++ -std=c++17 -O2 -Wall -Wextra -Werror -pedantic \
  -I native/src/jikl_fc01_system/include -I native/tests \
  native/tests/test_stage3_publication_combinations.cpp \
  -o /tmp/fc01-stage2-reset-publication
/tmp/fc01-stage2-reset-publication > /tmp/fc01-stage2-reset-publication.log

g++ -std=c++17 -O2 -Wall -Wextra -Werror -pedantic \
  -I native/src/jikl_fc01_system/include -I native/tests \
  native/tests/test_attitude_vertical.cpp -o /tmp/fc01-stage2-reset-vertical
/tmp/fc01-stage2-reset-vertical

cd native/tests
python3 -m unittest test_receive_event_adapter -v
```

结果：

1. 发布组合：编译/运行退出 0；日志末行 `Stage3 publication combinations: 522 PASS`。覆盖 8^3 上下文组合及 10 项异常/资源注入，早先拒绝不能被健康后观察覆盖，第三次拒绝必须报告部分发布，目标/心跳抛错不伪造提交完整。
2. 姿态/垂向：编译/运行退出 0；`attitude/vertical/ownership tests: PASS (quaternion, resets, signs, feed-forward, limits, anti-windup, faults)`。包括未活动 reset 可重建、活动 reset 保留旧参考并拒绝、无效四元数、垂向有效位/reset/NaN/Inf/包络。
3. 真实 SystemNode 适配器夹具：1 个 Python unittest、内部多个 C++ 断言场景，`Ran 1 test in 2.626s; OK`。覆盖好/坏/好姿态 reset 队列、无效姿态、安全事件溢出、图冲突、速度差仅诊断继续发布、真正时间回退/垂向 reset/位置残差异常仍阻断。

上述是测试程序/场景的不同计数，不能相加成单一测试数量。没有实测 DDS 调度、网络投递、PX4 Failsafe 延迟、姿态增量连续补偿或新硬件飞行。

## 风险分级和阶段 3 待证

- **P1 审查事项，非新确认缺陷**：活动姿态重置会停止后续 FC01 目标/心跳，最终退出由 PX4 既有失联保护处理。保留现有门；后续以同次 delta_q_reset、q、counter、发布审计、PX4 最后心跳、模式/failsafe 事件评估参考连续性和退出合同。
- **P2 可观测性缺口**：当前部分排队事件缺独立源 timestamp 和触发位细分，旧报告只保留通用 queued 原因；这阻碍定位，不能据旧健康尾帧反推首因。
- **现有合理保护**：真正 reset、无效姿态、Kill、控制权变化、过期反馈、部分发布不能用普通健康消息或补心跳掩盖。
- 未确认新 P0；本子任务无生产补丁，没有将待证推测混入飞行行为。

阶段 3 可直接复用上面三条离线回归命令。板端需从任务 READY 前开始保留完整 controller.log/journal 与 PX4 ULog，记录同一 boot/PID/发行身份及时间对应点；核对 `delta_q_reset`、quat_reset_counter、z/vz reset、最后收到 OffboardControlMode、COM_OF_LOSS_T/COM_OBL_RC_ACT。不要以最后日志行代替最后网络包。
