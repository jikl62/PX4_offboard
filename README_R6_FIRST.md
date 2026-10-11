> 历史 R6 基线记录，保留原结论与来源。本文件不是 R6.1 的当前结论；请先读根目录 `README_FIRST.md` 和 `docs/R6_1_SAFETY_EVIDENCE.md`。

# R6 先读此文件

版本 `0.4.4-offboard-flight-dev-r6-control-recovery-dev`，来源为交接包内完整 R5 独立副本。

**仅离线 DEV / BLOCKED。没有真机飞行放行，也没有替换用户现有 R4/R5。**

下降超时进入有限 RecoveryDescent，不再立即切断健康 Armed+Offboard 双 sink；
可选 Land21 由前置证据与当前完整本地位置共同门控，默认关闭。
恢复预算耗尽而无合格独立后备时，仍不能保证空中控制权：新延长 C 检查仍 RED。

阅读 `docs/R6_CHANGELOG.md`、`docs/R6_SAFETY_CLAIMS_AND_EVIDENCE.md`、`docs/C_REVIEW_TASK.md`。
历史 R4/R5 报告留作来源，不是本版本当前通过结论。

仅离线运行：

```bash
bash scripts/test.sh
bash scripts/r6_no_props_check.sh
```

第二个脚本只检查本地配置；也可加 `--events 已采集的JSONL文件` 审核本地提交记录。
它不启动控制器、不连接设备、不发送解锁/模式/参数命令。

有已安装 ROS2 Humble 的隔离构建环境时，仅编译源码：

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths workspace/src/px4_msgs src/jikl_fc01_system \
  --packages-select px4_msgs jikl_fc01_system
```

本轮未执行上述 native build；不得使用旧 `BASELINE_COLCON_BUILD.log` 替代本版本构建证据。
真机部署与控制命令待 A 适配、原生构建、C 针对本源码/哈希的独立分项评估。
